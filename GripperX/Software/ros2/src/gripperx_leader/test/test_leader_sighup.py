"""SIGHUP runs leader_arm_node's shutdown() (audit M2, 2026-10-16).

Closing the terminal that started the node (gripperx_desk.sh's or a hand start's)
sends SIGHUP. Python's default for it kills the process without the finally block in
main(): no wall release, no alignment abort, a servo held by the wall stays torqued.
main() runs here in a child process with LeaderArmNode replaced by a stand-in (no
bus), and the test asserts that SIGHUP reaches the stand-in's shutdown().
"""

import os
import signal
import subprocess
import sys
import time

CHILD = r'''
import os, sys
import rclpy
from rclpy.node import Node
from gripperx_leader import leader_arm_node as lan

class StandIn(Node):
    def __init__(self):
        super().__init__('leader_sighup_standin')
        self.create_timer(0.03, lambda: None)      # like the node's rate_hz tick
        print('READY', flush=True)

    def shutdown(self):
        print('SHUTDOWN STARTED', flush=True)
        import time
        time.sleep(float(os.environ.get('STANDIN_SHUTDOWN_SEC', '0')))   # a slow torque-off
        print('SHUTDOWN RAN', flush=True)

lan.LeaderArmNode = StandIn
lan.main()
print('MAIN RETURNED', flush=True)
'''


def _env():
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '77')
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
    env.pop('ROS_LOCALHOST_ONLY', None)
    env.pop('ROS_STATIC_PEERS', None)
    return env


def _run_and_signal(sig, second=None, shutdown_sec=0.0):
    env = _env()
    env['STANDIN_SHUTDOWN_SEC'] = str(shutdown_sec)
    proc = subprocess.Popen([sys.executable, '-c', CHILD], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env)
    try:
        deadline = time.monotonic() + 20.0
        line = ''
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if 'READY' in line or not line:
                break
        assert 'READY' in line, 'stand-in node did not start'
        time.sleep(0.3)
        proc.send_signal(sig)
        if second is not None:
            line = proc.stdout.readline()
            while line and 'SHUTDOWN STARTED' not in line:
                line = proc.stdout.readline()
            proc.send_signal(second)               # while shutdown() is still running
        out, _ = proc.communicate(timeout=10.0)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    return proc.returncode, out


def test_sighup_runs_shutdown():
    rc, out = _run_and_signal(signal.SIGHUP)
    assert 'SHUTDOWN RAN' in out, out
    assert rc == 0, out


def test_sigint_still_runs_shutdown():
    rc, out = _run_and_signal(signal.SIGINT)
    assert 'SHUTDOWN RAN' in out, out


def test_second_signal_during_shutdown_does_not_abort_it():
    """Re-audit DL1b: HUP HUP (or INT then HUP) used to raise into shutdown()."""
    for first, second in ((signal.SIGHUP, signal.SIGHUP), (signal.SIGINT, signal.SIGHUP),
                          (signal.SIGHUP, signal.SIGINT)):
        rc, out = _run_and_signal(first, second, shutdown_sec=1.0)
        assert 'SHUTDOWN RAN' in out, (first, second, out)
        assert rc == 0, (first, second, out)
