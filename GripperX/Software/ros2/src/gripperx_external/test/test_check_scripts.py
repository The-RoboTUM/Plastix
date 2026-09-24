"""Run this package's `check_*.py` scripts under pytest, so `colcon test` sees them.

WHY THIS FILE EXISTS. The checks are standalone scripts with a `__main__` block
and no `def test_*` functions, and they are named `check_*.py` rather than
`test_*.py`. pytest's default collection pattern therefore matched none of them:
`colcon test` on this package reported "NO TESTS RAN" and exited green. A suite
that cannot fail is not a check, so this wrapper runs each script as its own
process and asserts it exits 0 -- which is the contract the scripts already have.

ROS_DOMAIN_ID is forced to 220 (a twin domain). Several of these refuse to start
on an unknown domain, because a twin configuration on a real robot's domain
could reach real motors.

THE EXCLUSIONS BELOW ARE DELIBERATE AND ARE NOT A TODO LIST. Read the comment on
each before adding it: an automated suite must not command motion, and it must
not depend on a simulator nobody started.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).parent

CHECKS = [
    "check_frame_gate.py",
    "check_geodesy.py",
    "check_grasp.py",
    "check_rosbridge_client.py",
    "check_transform_status.py",
    "check_validation.py",
]

# EXCLUDED: check_stage3_twin.py -- an integration check that needs the twin
# actually running on ROS_DOMAIN_ID=221. It refuses to start otherwise, by
# design, so running it here would only assert that nobody launched Gazebo.


@pytest.mark.parametrize("script", CHECKS)
def test_check_script(script):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = "220"
    result = subprocess.run(
        [sys.executable, str(_HERE / script)],
        env=env, capture_output=True, text=True, timeout=300,
    )
    if result.returncode != 0:
        pytest.fail(
            f"{script} exited {result.returncode}\n"
            f"--- stdout ---\n{result.stdout[-4000:]}\n"
            f"--- stderr ---\n{result.stderr[-4000:]}"
        )
