"""Teleop shutdown: the stop goes out first, then the process exits cleanly.

Regression for 2026-10-22: quitting the web teleop from the UI ended in
`terminate called without an active exception` (SIGABRT, exit -6). The cause
was the daemon spin thread: main() returned while that thread was still inside
an rclpy C++ call, the interpreter finalised underneath it, and the thread's
forced unwind through the C++ frame called std::terminate. The keyboard node
had the same main() shape.

These tests run the REAL nodes as subprocesses on a private, localhost-only
ROS domain and a free TCP port -- nothing here can reach the robot -- and check:

* exit code 0, no "terminate called" in the output;
* the shutdown is logged as a shutdown, not as an EMERGENCY STOP (the user read
  the old line as an e-stop that had fired);
* the stop (/arm/stop and a zero Twist) was published after the quit and
  before the process ended -- the safety order survives the fix;
* the desk quit flag is written on a deliberate quit and NOT on SIGINT.
"""
import json
import os
import pty
import select
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Empty

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS = os.path.join(PKG, 'config', 'keyboard_teleop.yaml')
DOMAIN_ID = 197   # private: never 20, the robot's domain
CMD_VEL = '/teleop/keyboard/cmd_vel'
STARTUP_TIMEOUT = 20.0
EXIT_TIMEOUT = 10.0


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
    """Listens on the private domain for the stop the node must publish."""

    def __init__(self):
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=DOMAIN_ID)
        self.node = rclpy.create_node('teleop_shutdown_observer', context=self.ctx)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.twists = []      # (monotonic time, is_zero)
        self.arm_stops = []   # monotonic time
        self.node.create_subscription(
            Twist, CMD_VEL,
            lambda m: self.twists.append((time.monotonic(), _is_zero(m))), 10)
        self.node.create_subscription(
            Empty, '/arm/stop', lambda _m: self.arm_stops.append(time.monotonic()), 10)

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


def _post(port, body):
    req = urllib.request.Request(
        f'http://127.0.0.1:{port}/api/input',
        data=json.dumps(body).encode(), method='POST',
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=2.0) as r:
        return json.loads(r.read())


def _assert_clean(code, output):
    assert 'terminate called' not in output, output
    assert code == 0, f'exit code {code}\n{output}'


def _assert_stop_between(observer, t_quit, t_exit):
    assert any(t_quit <= t <= t_exit for t in observer.arm_stops), \
        'no /arm/stop between the quit and the exit'
    assert any(z and t_quit <= t <= t_exit for t, z in observer.twists), \
        'no zero Twist between the quit and the exit'


# -- web teleop ---------------------------------------------------------------

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
    # Server up AND the observer matched to the node (its publish timer feeds
    # cmd_vel continuously), so the stop cannot be lost to discovery.
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
        observer.executor.spin_once(timeout_sec=0.02)
    t_exit = time.monotonic()
    if proc.poll() is None:
        proc.kill()
        pytest.fail('web teleop did not exit\n' + proc.communicate()[0])
    return t_exit, proc.communicate()[0]


def test_web_quit_from_ui_exits_cleanly_after_the_stop(observer, tmp_path):
    proc, port, flag = _start_web(tmp_path)
    try:
        _wait_web_ready(proc, port, observer)
        assert _post(port, {'session': 't', 'claim': True,
                            'keys': [], 'events': []})['control']
        t_quit = time.monotonic()
        _post(port, {'session': 't', 'keys': [], 'events': ['quit|1|0']})
        t_exit, out = _wait_web_exit(proc, observer)
    finally:
        if proc.poll() is None:
            proc.kill()
    _assert_clean(proc.returncode, out)
    assert 'shutdown: stop + center + keyboard mode' in out, out
    assert 'EMERGENCY STOP' not in out, out
    _assert_stop_between(observer, t_quit, t_exit)
    assert os.path.exists(flag), 'deliberate quit did not signal the desk'


@pytest.mark.parametrize('sig,group', [
    (signal.SIGINT, False), (signal.SIGTERM, False), (signal.SIGINT, True),
], ids=['SIGINT', 'SIGTERM', 'SIGINT_process_group'])
def test_web_signal_exits_cleanly_without_desk_quit(observer, tmp_path, sig, group):
    # The process-group SIGINT is what Ctrl+C in the launching terminal sends.
    # That the stop is published on these paths is test_teleop_signal_stop.py.
    proc, port, flag = _start_web(tmp_path)
    try:
        _wait_web_ready(proc, port, observer)
        if group:
            os.killpg(proc.pid, sig)
        else:
            proc.send_signal(sig)
        _, out = _wait_web_exit(proc, observer)
    finally:
        if proc.poll() is None:
            proc.kill()
    _assert_clean(proc.returncode, out)
    assert 'EMERGENCY STOP' not in out, out
    assert not os.path.exists(flag), 'a signal must not tear the desk down'


# -- terminal (keyboard) teleop -----------------------------------------------

def _run_keyboard(tmp_path, observer, keys):
    """Run the terminal node on a pty (it reads /dev/tty) and type `keys`."""
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

    def drain(wait):
        while True:
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
    t_quit = t_exit = 0.0
    try:
        end = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < end and not (
                observer.twists and 'release reporting' in ''.join(chunks)):
            drain(0.05)
            observer.spin_for(0.05)
        assert observer.twists, 'keyboard teleop did not come up\n' + ''.join(chunks)
        t_quit = time.monotonic()
        os.write(fd, keys)
        end = time.monotonic() + EXIT_TIMEOUT
        while time.monotonic() < end:
            drain(0.02)
            observer.spin_for(0.02)
            done, st = os.waitpid(pid, os.WNOHANG)
            if done:
                status = st
                break
        t_exit = time.monotonic()
        drain(0.2)
    finally:
        if status is None:
            try:
                os.kill(pid, signal.SIGKILL)
                os.waitpid(pid, 0)
            except (ProcessLookupError, ChildProcessError):
                pass
        os.close(fd)
    out = ''.join(chunks)
    assert status is not None, 'keyboard teleop did not exit\n' + out
    return os.waitstatus_to_exitcode(status), out, t_quit, t_exit, flag


# forkpty() in a multi-threaded process (the observer's DDS threads) warns about
# deadlocks in the child; the child only execs, so the warning does not apply.
@pytest.mark.filterwarnings('ignore:This process .* is multi-threaded')
@pytest.mark.parametrize('keys', [b'q', b'\x03'], ids=['Q', 'ctrl_c_byte'])
def test_keyboard_quit_exits_cleanly_after_the_stop(observer, tmp_path, keys):
    # In raw mode Ctrl+C arrives as the byte 0x03, not as a signal.
    code, out, t_quit, t_exit, flag = _run_keyboard(tmp_path, observer, keys)
    _assert_clean(code, out)
    assert 'shutdown: stop + center + keyboard mode' in out, out
    assert 'EMERGENCY STOP' not in out, out
    _assert_stop_between(observer, t_quit, t_exit)
    assert os.path.exists(flag), 'deliberate quit did not signal the desk'
