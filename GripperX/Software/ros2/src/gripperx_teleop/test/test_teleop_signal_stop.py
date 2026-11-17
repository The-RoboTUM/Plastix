"""Every way a teleop node ends publishes the explicit stop first.

Follow-up to test_teleop_shutdown.py (2026-10-22 audit of the exit-abort fix):
on SIGINT, SIGTERM and SIGHUP the nodes used to end WITHOUT the explicit stop --
rclpy's own signal handler took the ROS context down before main() could
publish, or a KeyboardInterrupt skipped the shutdown tail -- and the keyboard
node hung on SIGTERM with the terminal left raw. gripperx_desk.sh tears the
teleop down with SIGINT, then SIGTERM after 3 s, then SIGKILL after 2 s more.

For every end path of both nodes this checks, between the trigger and the exit:

* /arm/stop arrived (only center() publishes it -- the zero Twist is published
  continuously and proves nothing on its own);
* straight steering [0, 0, 0, 0] on /teleop/direct_steer;
* mode 'keyboard' on /teleop/set_mode;
* leader disarm (False on /arm/leader/enable);
* exit code 0 within 3 s, no "terminate called", no Traceback;
* the desk quit flag only on a deliberate quit.

The real nodes run as subprocesses on a private, localhost-only ROS domain and a
free TCP port -- nothing here can reach the robot.
"""
import http.client
import json
import os
import pty
import select
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request

import pytest
import rclpy
from geometry_msgs.msg import Twist
from gripperx_arm_msgs.action import PickPlastic
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64MultiArray, Int64, String

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS = os.path.join(PKG, 'config', 'keyboard_teleop.yaml')
DOMAIN_ID = 194   # private: never 20, the robot's domain
CMD_VEL = '/teleop/keyboard/cmd_vel'
STARTUP_TIMEOUT = 20.0
EXIT_TIMEOUT = 8.0
# gripperx_desk.sh escalates to SIGTERM 3 s after its SIGINT.
EXIT_BUDGET = 3.0
STEER_JOINTS = 4


def _free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _node_env(flag_path):
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(DOMAIN_ID)
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['PYTHONPATH'] = PKG + os.pathsep + env.get('PYTHONPATH', '')
    env['PYTHONUNBUFFERED'] = '1'
    env['GRIPPERX_DESK_QUIT_FLAG'] = flag_path
    return env


def _is_zero(t):
    return not any((t.linear.x, t.linear.y, t.linear.z,
                    t.angular.x, t.angular.y, t.angular.z))


class _Observer:
    """Records, with arrival time, everything the stop consists of."""

    def __init__(self):
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=DOMAIN_ID)
        self.node = rclpy.create_node('teleop_signal_stop_observer', context=self.ctx)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.twists = []       # (t, is_zero)
        self.arm_stops = []    # t
        self.steers = []       # (t, data)
        self.modes = []        # (t, mode)
        self.leader = []       # (t, enabled)
        # Publisher-side send time (ns, wall clock, same machine): the order in
        # which two topics' callbacks run here is not the order they were sent.
        self.twist_src = []     # (source ns, is_zero)
        self.arm_stop_src = []  # source ns
        now = time.monotonic

        def on_twist(m, info):
            self.twists.append((now(), _is_zero(m)))
            self.twist_src.append((info['source_timestamp'], _is_zero(m)))

        def on_arm_stop(_m, info):
            self.arm_stops.append(now())
            self.arm_stop_src.append(info['source_timestamp'])
        self.node.create_subscription(Twist, CMD_VEL, on_twist, 200)
        self.node.create_subscription(Empty, '/arm/stop', on_arm_stop, 10)
        self.node.create_subscription(
            Float64MultiArray, '/teleop/direct_steer',
            lambda m: self.steers.append((now(), list(m.data))), 10)
        self.mode_src = []      # (source ns, mode)

        def on_mode(m, info):
            self.modes.append((now(), m.data))
            self.mode_src.append((info['source_timestamp'], m.data))
        self.node.create_subscription(String, '/teleop/set_mode', on_mode, 10)
        self.node.create_subscription(
            Bool, '/arm/leader/enable', lambda m: self.leader.append((now(), m.data)), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def observer(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    obs = _Observer()
    yield obs
    obs.close()


def _assert_explicit_stop(observer, t0, t1):
    def within(rows):
        return [r for r in rows if t0 <= r[0] <= t1]
    missing = []
    if not any(t0 <= t <= t1 for t in observer.arm_stops):
        missing.append('/arm/stop')
    if not any(d == [0.0] * STEER_JOINTS for _, d in within(observer.steers)):
        missing.append('straight steer on /teleop/direct_steer')
    if not any(m == 'keyboard' for _, m in within(observer.modes)):
        missing.append("mode 'keyboard'")
    if not any(e is False for _, e in within(observer.leader)):
        missing.append('leader disarm')
    assert not missing, 'missing between trigger and exit: ' + ', '.join(missing)


def _assert_clean_exit(code, out, t0, t1, check_output=True):
    assert code == 0, f'exit code {code}\n{out}'
    assert t1 - t0 <= EXIT_BUDGET, f'exit took {t1 - t0:.2f} s\n{out}'
    if check_output:
        assert 'terminate called' not in out, out
        assert 'Traceback' not in out, out
        assert 'EMERGENCY STOP' not in out, out
        assert 'shutdown: stop + center + keyboard mode' in out, out


# -- web teleop ---------------------------------------------------------------

def _post(port, body):
    req = urllib.request.Request(
        f'http://127.0.0.1:{port}/api/input',
        data=json.dumps(body).encode(), method='POST',
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=2.0) as r:
        return json.loads(r.read())


def _start_web(tmp_path):
    port = _free_port()
    flag = str(tmp_path / 'desk_quit.flag')
    proc = subprocess.Popen(
        [sys.executable, '-m', 'gripperx_teleop.web_teleop_node',
         '--ros-args', '--params-file', PARAMS,
         '-p', 'web_host:=127.0.0.1', '-p', f'web_port:={port}',
         '-p', 'open_browser:=false'],
        env=_node_env(flag), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    return proc, port, flag


def _wait_web_ready(proc, port, observer):
    def up():
        try:
            urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=0.5).close()
            return True
        except OSError:
            return False
    end = time.monotonic() + STARTUP_TIMEOUT
    while time.monotonic() < end and proc.poll() is None:
        if observer.twists and up():
            return
        observer.spin_for(0.1)
    proc.kill()
    pytest.fail('web teleop did not come up\n' + proc.communicate()[0])


def _wait_web_exit(proc, observer):
    end = time.monotonic() + EXIT_TIMEOUT
    while proc.poll() is None and time.monotonic() < end:
        observer.executor.spin_once(timeout_sec=0.01)
    t_exit = time.monotonic()
    if proc.poll() is None:
        proc.kill()
        pytest.fail('web teleop did not exit\n' + proc.communicate()[0])
    observer.spin_for(0.1)
    return t_exit, proc.communicate()[0]


def _web_case(observer, tmp_path, trigger):
    proc, port, flag = _start_web(tmp_path)
    try:
        _wait_web_ready(proc, port, observer)
        assert _post(port, {'session': 't', 'claim': True,
                            'keys': [], 'events': []})['control']
        observer.spin_for(0.3)
        t0 = time.monotonic()
        trigger(proc, port)
        t1, out = _wait_web_exit(proc, observer)
    finally:
        if proc.poll() is None:
            proc.kill()
    return proc.returncode, out, t0, t1, flag


def _ui_quit(_proc, port):
    _post(port, {'session': 't', 'keys': [], 'events': ['quit|1|0']})


def _signal(sig, group=False):
    def trigger(proc, _port):
        if group:
            os.killpg(proc.pid, sig)
        else:
            proc.send_signal(sig)
    return trigger


WEB_SIGNALS = [
    ('SIGINT', _signal(signal.SIGINT)),
    ('SIGTERM', _signal(signal.SIGTERM)),
    ('SIGHUP', _signal(signal.SIGHUP)),
    ('SIGINT_process_group', _signal(signal.SIGINT, group=True)),
]


def test_web_ui_quit_stops_and_signals_desk(observer, tmp_path):
    code, out, t0, t1, flag = _web_case(observer, tmp_path, _ui_quit)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    assert os.path.exists(flag), 'deliberate quit did not signal the desk'


@pytest.mark.parametrize('trigger', [t for _, t in WEB_SIGNALS],
                         ids=[n for n, _ in WEB_SIGNALS])
def test_web_signal_publishes_stop_before_exit(observer, tmp_path, trigger):
    code, out, t0, t1, flag = _web_case(observer, tmp_path, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    assert not os.path.exists(flag), 'a signal must not tear the desk down'


def test_web_desk_escalation_does_not_abort_the_stop(observer, tmp_path):
    # SIGINT, then SIGTERM straight into the shutdown tail: the second signal
    # must not cut the stop short.
    def trigger(proc, _port):
        proc.send_signal(signal.SIGINT)
        time.sleep(0.05)
        proc.send_signal(signal.SIGTERM)
    code, out, t0, t1, flag = _web_case(observer, tmp_path, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    assert not os.path.exists(flag)


def _assert_one_stop_burst(observer, t0, t1):
    n = sum(1 for t in observer.arm_stops if t0 <= t <= t1)
    assert n == 1, f'{n} /arm/stop between trigger and exit, expected one burst'


def test_web_double_sigint_one_full_stop(observer, tmp_path):
    # Ctrl-C in the desk terminal reaches the node, and the desk's own teardown
    # SIGINT lands during the shutdown tail: the tail must run once, whole.
    def trigger(proc, _port):
        proc.send_signal(signal.SIGINT)
        time.sleep(0.02)
        proc.send_signal(signal.SIGINT)
    code, out, t0, t1, flag = _web_case(observer, tmp_path, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    _assert_one_stop_burst(observer, t0, t1)
    assert out.count('shutdown: stop + center + keyboard mode') == 1, out
    assert not os.path.exists(flag)


def test_web_quit_with_a_key_held_ends_on_zero(observer, tmp_path):
    """The last Twist before the exit is zero even if the page still held W.

    The page's held keys are re-pressed by a timer until the link times out; a
    quit that arrives with W still held must not let that timer drive again
    after center() has published the stop.
    """
    proc, port, flag = _start_web(tmp_path)
    try:
        _wait_web_ready(proc, port, observer)
        assert _post(port, {'session': 't', 'claim': True,
                            'keys': [], 'events': []})['control']
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            _post(port, {'session': 't', 'keys': ['w'], 'events': []})
            observer.spin_for(0.05)
            if any(not z for _, z in observer.twists):
                break
        if not any(not z for _, z in observer.twists):
            pytest.skip('W does not drive in this offline setup')
        t0 = time.monotonic()
        _post(port, {'session': 't', 'keys': ['w'], 'events': ['quit|7|0']})
        t1, out = _wait_web_exit(proc, observer)
    finally:
        if proc.poll() is None:
            proc.kill()
    _assert_clean_exit(proc.returncode, out, t0, t1)
    _assert_no_drive_after_stop(observer)


def _assert_no_drive_after_stop(observer):
    """Ordered by send time: nothing but zero Twists after the first /arm/stop."""
    assert observer.arm_stop_src, 'no /arm/stop'
    first = min(observer.arm_stop_src)
    after = [(s - first) / 1e9 for s, z in observer.twist_src if s >= first]
    driving = [(s - first) / 1e9 for s, z in observer.twist_src
               if s >= first and not z]
    assert after, 'no Twist at all after the stop'
    assert not driving, \
        'non-zero Twist after the stop at +' + ', +'.join(f'{d:.3f}' for d in driving) + ' s'


class _KeepAliveHammer:
    """A browser-like page: ONE persistent HTTP/1.1 connection, W held, 20 Hz.

    A keep-alive connection outlives the server's listening socket, so this is
    the input that can still arrive after stop_server().
    """

    def __init__(self, port):
        self.conn = http.client.HTTPConnection('127.0.0.1', port, timeout=2.0)
        self.running = True
        self.posts = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def post(self, body):
        self.conn.request('POST', '/api/input', body=json.dumps(body),
                          headers={'Content-Type': 'application/json'})
        r = self.conn.getresponse()
        return json.loads(r.read())

    def _run(self):
        while self.running:
            try:
                self.post({'session': 't', 'keys': ['w'], 'events': []})
                self.posts += 1
            except (OSError, http.client.HTTPException, ValueError):
                return
            time.sleep(0.05)

    def stop(self):
        self.running = False
        self.thread.join(2.0)
        self.conn.close()


def _keepalive_case(observer, tmp_path, trigger):
    proc, port, flag = _start_web(tmp_path)
    hammer = None
    try:
        _wait_web_ready(proc, port, observer)
        hammer = _KeepAliveHammer(port)
        assert hammer.post({'session': 't', 'claim': True,
                            'keys': [], 'events': []})['control']
        hammer.thread.start()
        end = time.monotonic() + 3.0
        while time.monotonic() < end and all(z for _, z in observer.twists[-5:]):
            observer.spin_for(0.05)
        if all(z for _, z in observer.twists[-5:]):
            pytest.skip('W does not drive in this offline setup')
        t0 = time.monotonic()
        t0_ns = time.time_ns()
        trigger(proc, port)
        t1, out = _wait_web_exit(proc, observer)
        posts_at_exit = hammer.posts
    finally:
        if hammer is not None:
            hammer.stop()
        if proc.poll() is None:
            proc.kill()
    return proc.returncode, out, t0, t1, t0_ns, flag, posts_at_exit


def test_web_keepalive_w_then_sigint_ends_on_zero(observer, tmp_path):
    code, out, t0, t1, t0_ns, flag, _ = _keepalive_case(
        observer, tmp_path, _signal(signal.SIGINT))
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    _assert_no_drive_after_stop(observer)
    latency = (min(observer.arm_stop_src) - t0_ns) / 1e9
    print(f'web SIGINT -> /arm/stop: {latency * 1000:.1f} ms')
    assert latency <= 0.1, f'/arm/stop {latency:.3f} s after the signal'
    assert not os.path.exists(flag)


def test_web_keepalive_w_then_ui_quit_ends_on_zero(observer, tmp_path):
    # The quit comes from a second connection while the first still holds W.
    code, out, t0, t1, t0_ns, flag, _ = _keepalive_case(
        observer, tmp_path,
        lambda _p, port: _post(port, {'session': 't', 'keys': ['w'],
                                      'events': ['quit|9|0']}))
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    _assert_no_drive_after_stop(observer)
    assert os.path.exists(flag), 'deliberate quit did not signal the desk'


@pytest.mark.parametrize('trigger', [t for _, t in WEB_SIGNALS],
                         ids=[n for n, _ in WEB_SIGNALS])
def test_web_signal_stop_latency(observer, tmp_path, trigger):
    proc, port, _flag = _start_web(tmp_path)
    try:
        _wait_web_ready(proc, port, observer)
        observer.spin_for(0.3)
        t0_ns = time.time_ns()
        trigger(proc, port)
        _wait_web_exit(proc, observer)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert observer.arm_stop_src, 'no /arm/stop'
    latency = (min(observer.arm_stop_src) - t0_ns) / 1e9
    print(f'web signal -> /arm/stop: {latency * 1000:.1f} ms')
    assert latency <= 0.1, f'/arm/stop {latency:.3f} s after the signal'


def test_web_pick_in_flight_does_not_delay_or_follow_the_stop(observer, tmp_path):
    """P blocks its input thread in wait_for_server() (up to 0.5 s) while no arm
    action server is up. SIGINT lands during that wait; the server appears only
    after the stop, so the waiting pick finds it. Required: the stop is not
    delayed by the waiting pick, no goal goes out after the stop, and the
    'mode_autonomous' queued behind the pick in the same request is not
    published after it either.
    """
    proc, port, flag = _start_web(tmp_path)
    goals = []   # wall-clock ns at which a goal request reached the server
    server = None
    posters = []
    try:
        _wait_web_ready(proc, port, observer)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        epoch_pub = observer.node.create_publisher(Int64, '/arm/stop_epoch', latched)
        epoch_pub.publish(Int64(data=1))   # without an epoch P never sends
        assert _post(port, {'session': 't', 'claim': True,
                            'keys': [], 'events': []})['control']
        observer.spin_for(0.5)

        def poster(first_id):
            try:
                _post(port, {'session': 't', 'keys': [],
                             'events': [f'pick|{first_id}|0',
                                        f'mode_autonomous|{first_id + 1}|0']})
            except OSError:
                pass   # the server may be gone by the time this returns
        for first_id in (100, 200):
            posters.append(threading.Thread(target=poster, args=(first_id,), daemon=True))
            posters[-1].start()
        # rclpy's wait_for_server(0.5) looks at 0, 0.25 and 0.5 s. Signal at
        # ~0.2 s: a drain that waits for the picks (the old 0.2 s) published the
        # stop at ~0.4 s, so the 0.5 s look -- and the mode_autonomous queued
        # behind it -- lands after the stop while the context is still up.
        observer.spin_for(0.2)
        t0 = time.monotonic()
        t0_ns = time.time_ns()
        proc.send_signal(signal.SIGINT)
        # Bring the server up after that stop but before the 0.5 s look, so a
        # pick that ignores the gate finds it and sends.
        observer.spin_for(0.21)

        def on_goal(_req):
            goals.append(time.time_ns())
            return GoalResponse.REJECT
        server = ActionServer(observer.node, PickPlastic, 'pick_plastic',
                              execute_callback=lambda gh: PickPlastic.Result(),
                              goal_callback=on_goal)
        t1, out = _wait_web_exit(proc, observer)
        observer.spin_for(0.5)
    finally:
        if proc.poll() is None:
            proc.kill()
        for t in posters:
            t.join(2.0)
        if server is not None:
            server.destroy()
    _assert_clean_exit(proc.returncode, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    first = min(observer.arm_stop_src)
    latency = (first - t0_ns) / 1e9
    print(f'web SIGINT with picks in flight -> /arm/stop: {latency * 1000:.1f} ms')
    assert latency <= 0.1, f'/arm/stop {latency:.3f} s after the signal'
    late_goals = [(g - first) / 1e9 for g in goals if g >= first]
    assert not late_goals, f'pick goal(s) after the stop at +{late_goals} s'
    late_modes = [(m, (s - first) / 1e9) for s, m in observer.mode_src
                  if s >= first and m != 'keyboard']
    assert not late_modes, f'mode switch after the stop: {late_modes}'
    assert not os.path.exists(flag)


# -- terminal (keyboard) teleop -----------------------------------------------

def _run_keyboard(tmp_path, observer, trigger, read_output=True):
    """Run the terminal node on a pty (it reads /dev/tty) and fire `trigger`.

    trigger(pid, fd) returns False when it closed the pty master itself (a
    closed terminal window); the output is then gone with it.
    """
    flag = str(tmp_path / 'desk_quit.flag')
    env = _node_env(flag)
    pid, fd = pty.fork()
    if pid == 0:   # child: the pty is its controlling terminal
        try:
            os.execvpe(sys.executable,
                       [sys.executable, '-m', 'gripperx_teleop.keyboard_teleop_node',
                        '--ros-args', '--params-file', PARAMS], env)
        finally:
            os._exit(127)
    chunks = []
    fd_open = True

    def drain(wait):
        while fd_open:
            r, _, _ = select.select([fd], [], [], wait)
            if not r:
                return
            try:
                data = os.read(fd, 65536)
            except OSError:
                return
            if not data:
                return
            chunks.append(data.decode('utf-8', 'replace'))
            wait = 0.0

    status = None
    t0 = t1 = 0.0
    try:
        end = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < end and not (
                observer.twists and 'release reporting' in ''.join(chunks)):
            drain(0.05)
            observer.spin_for(0.05)
        assert observer.twists, 'keyboard teleop did not come up\n' + ''.join(chunks)
        observer.spin_for(0.3)
        t0 = time.monotonic()
        if trigger(pid, fd) is False:
            os.close(fd)
            fd_open = False
        end = time.monotonic() + EXIT_TIMEOUT
        while time.monotonic() < end:
            drain(0.01)
            observer.executor.spin_once(timeout_sec=0.01)
            done, st = os.waitpid(pid, os.WNOHANG)
            if done:
                status = st
                break
        t1 = time.monotonic()
        observer.spin_for(0.1)
        drain(0.2)
    finally:
        if status is None:
            try:
                os.kill(pid, signal.SIGKILL)
                os.waitpid(pid, 0)
            except (ProcessLookupError, ChildProcessError):
                pass
        if fd_open:
            os.close(fd)
    out = ''.join(chunks)
    assert status is not None, 'keyboard teleop did not exit\n' + out
    return os.waitstatus_to_exitcode(status), out, t0, t1, flag


def _keys(data):
    def trigger(_pid, fd):
        os.write(fd, data)
    return trigger


def _kill(sig):
    def trigger(pid, _fd):
        os.kill(pid, sig)
    return trigger


def _close_terminal(_pid, _fd):
    # Closing the master hangs the terminal up: SIGHUP to the child's session,
    # and every read of /dev/tty returns EOF/EIO from then on.
    return False


KB_DELIBERATE = [('Q', _keys(b'q')), ('ctrl_c_byte', _keys(b'\x03'))]
KB_SIGNALS = [('SIGINT', _kill(signal.SIGINT)),
              ('SIGTERM', _kill(signal.SIGTERM)),
              ('SIGHUP', _kill(signal.SIGHUP))]

# forkpty() in a multi-threaded process (the observer's DDS threads) warns about
# deadlocks in the child; the child only execs, so the warning does not apply.
_PTY_WARN = pytest.mark.filterwarnings('ignore:This process .* is multi-threaded')


@_PTY_WARN
@pytest.mark.parametrize('trigger', [t for _, t in KB_DELIBERATE],
                         ids=[n for n, _ in KB_DELIBERATE])
def test_keyboard_quit_stops_and_signals_desk(observer, tmp_path, trigger):
    code, out, t0, t1, flag = _run_keyboard(tmp_path, observer, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    assert 'Keyboard teleop ended.' in out, 'terminal not restored\n' + out
    assert os.path.exists(flag), 'deliberate quit did not signal the desk'


@_PTY_WARN
def test_keyboard_double_sigint_one_full_stop(observer, tmp_path):
    def trigger(pid, _fd):
        os.kill(pid, signal.SIGINT)
        time.sleep(0.02)
        os.kill(pid, signal.SIGINT)
    code, out, t0, t1, flag = _run_keyboard(tmp_path, observer, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    _assert_one_stop_burst(observer, t0, t1)
    assert out.count('shutdown: stop + center + keyboard mode') == 1, out
    assert 'Keyboard teleop ended.' in out, 'terminal not restored\n' + out
    assert not os.path.exists(flag)


@_PTY_WARN
@pytest.mark.parametrize('trigger', [t for _, t in KB_SIGNALS],
                         ids=[n for n, _ in KB_SIGNALS])
def test_keyboard_signal_publishes_stop_before_exit(observer, tmp_path, trigger):
    code, out, t0, t1, flag = _run_keyboard(tmp_path, observer, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    assert 'Keyboard teleop ended.' in out, 'terminal not restored\n' + out
    assert not os.path.exists(flag), 'a signal must not tear the desk down'


@_PTY_WARN
def test_keyboard_desk_escalation_does_not_abort_the_stop(observer, tmp_path):
    def trigger(pid, _fd):
        os.kill(pid, signal.SIGINT)
        time.sleep(0.05)
        os.kill(pid, signal.SIGTERM)
    code, out, t0, t1, flag = _run_keyboard(tmp_path, observer, trigger)
    _assert_clean_exit(code, out, t0, t1)
    _assert_explicit_stop(observer, t0, t1)
    assert not os.path.exists(flag)


@_PTY_WARN
def test_keyboard_closed_terminal_publishes_stop_before_exit(observer, tmp_path):
    # The output went with the terminal, so only the topics and the exit code
    # can be checked here.
    code, out, t0, t1, flag = _run_keyboard(tmp_path, observer, _close_terminal)
    _assert_clean_exit(code, out, t0, t1, check_output=False)
    _assert_explicit_stop(observer, t0, t1)
    assert not os.path.exists(flag), 'a hang-up must not tear the desk down'


# -- the gates themselves, in-process ------------------------------------------
# The subprocess test above cannot hold a pick at "server found, after the stop"
# reliably once the fix makes the process exit quickly; these do, deterministically:
# the shutdown begins INSIDE the pick's server wait, which then succeeds.

class _Sink:
    def __init__(self, log=None, topic=''):
        self.log, self.topic = log, topic

    def publish(self, msg):
        if self.log is not None:
            self.log.append((self.topic, getattr(msg, 'data', None)))


class _GateClient:
    """A pick client whose server wait lets `during_wait` run, then finds it."""

    def __init__(self, during_wait):
        self.during_wait = during_wait
        self.goals = []

    def wait_for_server(self, timeout_sec=0.0):
        self.during_wait()
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)

        class F:
            def add_done_callback(self, cb):
                pass
        return F()


@pytest.fixture
def gate_node():
    from gripperx_geometry.constants import (
        HALF_TRACK_KINGPIN as B, HALF_WHEELBASE_KINGPIN as A,
        WHEEL_RADIUS_EFFECTIVE as R)
    from gripperx_teleop import web_teleop_node as wtn
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(domain_id=DOMAIN_ID, args=[
        '--ros-args', '-p', f'a:={A}', '-p', f'b:={B}', '-p', f'wheel_radius:={R}'])
    n = wtn.WebTeleopNode()
    log = []
    n._leader_pub = n._arm_pub = n._arm_stop_pub = n._align_pub = _Sink()
    n._mode_pub = _Sink(log, 'mode')
    n._on_stop_epoch(Int64(data=1))
    yield n, log
    n.destroy_node()
    rclpy.shutdown()


def test_pick_waking_after_the_gate_sends_no_goal(gate_node):
    n, _log = gate_node
    n._pick_client = _GateClient(lambda: n.begin_ending(drain_timeout=0.0))
    assert n.trigger_pick() is False
    assert n._pick_client.goals == []
    assert not n._pick_busy


def test_pick_before_the_gate_still_sends(gate_node):
    n, _log = gate_node
    n._pick_client = _GateClient(lambda: None)
    assert n.trigger_pick() is True
    assert len(n._pick_client.goals) == 1


def test_one_shot_queued_behind_a_blocking_pick_is_dropped(gate_node):
    # The re-audit's case: [pick, mode_autonomous] in one request, the shutdown
    # begins while P waits for its server. Neither may go out afterwards.
    n, log = gate_node
    n._pick_client = _GateClient(lambda: n.begin_ending(drain_timeout=0.0))
    n._on_client_input('s', [], ['pick|1|0', 'mode_autonomous|2|0'], True)
    assert n._pick_client.goals == []
    assert ('mode', 'autonomous') not in log, log


def test_dispatch_after_the_gate_does_nothing(gate_node):
    n, log = gate_node
    n.begin_ending(drain_timeout=0.0)
    for name in ('mode_autonomous', 'mode_keyboard', 'home', 'leader', 'pick'):
        n._dispatch(name)
    assert log == []
