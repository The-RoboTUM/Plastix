#!/usr/bin/env python3
"""Fail if the active localization.yaml does not make slam_toolbox scan on rotation.

Without `check_min_dist_and_heading_precisely: true` slam_toolbox 2.8.5 checks
distance only and ignores minimum_travel_heading, so it processes no scan during
in-place rotation (2026-10-08 coast bag: 0 map->odom updates in all rotation
phases). Removing the key passes every other test; this one fails on it.

Usage: check_slam_rotation_scans.py [path/to/localization.yaml]
"""

import sys
from pathlib import Path

import yaml

_DEFAULT = Path(__file__).resolve().parent.parent / "config" / "localization.yaml"
KEY = "check_min_dist_and_heading_precisely"


def main(argv):
    path = Path(argv[1]) if len(argv) > 1 else _DEFAULT
    params = yaml.safe_load(path.read_text())["slam_toolbox"]["ros__parameters"]
    if params.get(KEY) is not True:
        print(f"FAIL {path}: slam_toolbox.{KEY} is {params.get(KEY, 'absent')!r}, "
              "expected true -- otherwise no scan is matched during in-place rotation.")
        return 1
    print(f"OK {path}: slam_toolbox.{KEY} is true")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
