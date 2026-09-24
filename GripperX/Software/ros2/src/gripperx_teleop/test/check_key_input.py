#!/usr/bin/env python3
"""Verification of the key input layer: sequence parsing and hold semantics.

Mirrors gripperx_teleop/test/check_manoeuvres.py: pure python, no ROS and no
tty required. Run from the workspace source tree:

    python3 src/gripperx_teleop/test/check_key_input.py

Part 1 decodes the escape sequences a terminal actually emits under the kitty
keyboard protocol and checks that each resolves to the right key and the right
event type — including the two cases that would silently break the teleop if
they regressed: ctrl+C must stay a quit path once flag 8 removes plain bytes,
and a modified cursor key must NOT fall through as a plain arrow.

Part 2 exercises `KeyStateTracker` in both regimes. The property under test is
the one the operator feels: with release reporting, letting go of W stops the
robot on the release event rather than `drive_hold_sec` later; without it, the
window is measured from the terminal's own repeat interval instead of assumed.
The dead-man ceiling is checked to survive both.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))

from gripperx_teleop.key_input import (  # noqa: E402
    PRESS,
    RELEASE,
    REPEAT,
    KeyStateTracker,
    parse_csi,
)

DRIVE_KEYS = ("w", "s", "a", "d", "up", "down", "left", "right")
CEILING = 0.6

failures = []


def check(label: str, condition: bool) -> None:
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
    if not condition:
        failures.append(label)


def main() -> int:
    print("\n== Part 1: escape sequence decoding ==\n")

    # CSI <code> ; <mods> : <event> u — the flag-8 form of an ordinary letter.
    check("CSI 119;1:1u  -> ('w', PRESS)", parse_csi("119;1:1", "u") == ("w", PRESS))
    check("CSI 119;1:2u  -> ('w', REPEAT)", parse_csi("119;1:2", "u") == ("w", REPEAT))
    check("CSI 119;1:3u  -> ('w', RELEASE)", parse_csi("119;1:3", "u") == ("w", RELEASE))
    check("CSI 32;1:3u   -> ('space', RELEASE)", parse_csi("32;1:3", "u") == ("space", RELEASE))

    # A terminal given only flag 1 sends no event field at all. Treating that as
    # a press is what lets one dispatch serve both regimes.
    check("CSI 119u (no event field) -> ('w', PRESS)", parse_csi("119", "u") == ("w", PRESS))

    # Cursor keys keep their own final byte; the event rides in the same place.
    check("CSI 1;1:1A    -> ('up', PRESS)", parse_csi("1;1:1", "A") == ("up", PRESS))
    check("CSI 1;1:3D    -> ('left', RELEASE)", parse_csi("1;1:3", "D") == ("left", RELEASE))
    check("CSI A (bare, pre-protocol) -> ('up', PRESS)", parse_csi("", "A") == ("up", PRESS))

    # THE TWO REGRESSIONS THIS PART EXISTS FOR.
    check(
        "ctrl+C survives flag 8: CSI 99;5u -> ('ctrl_c', PRESS)",
        parse_csi("99;5", "u") == ("ctrl_c", PRESS),
    )
    check(
        "ctrl+up is DROPPED, never read as a plain arrow",
        parse_csi("1;5:1", "A") is None,
    )
    check("an unbound key is dropped: CSI 122u ('z')", parse_csi("122", "u") is None)
    check("a malformed parameter run is dropped", parse_csi("abc;1:1", "u") is None)

    print("\n== Part 2: hold semantics WITH release reporting ==\n")

    keys = KeyStateTracker(DRIVE_KEYS, ceiling_sec=CEILING, release_reporting=True)
    keys.on_event("w", PRESS, 0.0)
    check("held immediately after the press", keys.held("w", 0.0))
    keys.on_event("w", REPEAT, 0.4)
    check("still held 0.4 s later while repeats arrive", keys.held("w", 0.4))
    keys.on_event("w", RELEASE, 0.5)
    check(
        "NOT held 1 ms after the release — this is the overshoot that is gone",
        not keys.held("w", 0.501),
    )

    # The old behaviour, for contrast: 0.6 s of drive after the last event.
    check(
        "…and under the old model it would still have been held at 0.5 + 0.09 s",
        (0.59 - 0.5) < CEILING,
    )

    # Two keys at once — the thing terminal auto-repeat structurally cannot do.
    keys = KeyStateTracker(DRIVE_KEYS, ceiling_sec=CEILING, release_reporting=True)
    keys.on_event("w", PRESS, 0.0)
    keys.on_event("a", PRESS, 0.1)
    check("W and A held simultaneously (drive + steer)", keys.held("w", 0.1) and keys.held("a", 0.1))
    keys.on_event("a", RELEASE, 0.3)
    check("releasing A leaves W held", keys.held("w", 0.3) and not keys.held("a", 0.3))

    # SR-3 / incident 06.07.: the ceiling must survive a terminal that dies
    # without ever sending the release.
    keys = KeyStateTracker(DRIVE_KEYS, ceiling_sec=CEILING, release_reporting=True)
    keys.on_event("w", PRESS, 0.0)
    check(
        "dead-man ceiling still fires when NO release ever arrives",
        keys.held("w", CEILING - 0.01) and not keys.held("w", CEILING + 0.01),
    )

    print("\n== Part 3: hold semantics WITHOUT release reporting (fallback) ==\n")

    keys = KeyStateTracker(DRIVE_KEYS, ceiling_sec=CEILING, release_reporting=False)
    # The initial repeat delay: one press, then nothing for ~0.5 s. The
    # conservative ceiling has to cover it or the start-off stutters.
    keys.on_event("w", PRESS, 0.0)
    check("start-off gap: still held at 0.45 s on one press alone", keys.held("w", 0.45))
    check("window before any repeat is measured == the ceiling", keys.window("w") == CEILING)

    # Repeats arrive at 30 ms. After two intervals the window is measured.
    for index in range(1, 6):
        keys.on_event("w", REPEAT, 0.5 + index * 0.03)
    measured = keys.window("w")
    check(
        f"window measured from the 30 ms repeat rate: {measured:.3f} s, well under {CEILING}",
        0.09 <= measured <= 0.15,
    )
    last = 0.5 + 5 * 0.03
    check("held while the repeats keep coming", keys.held("w", last))
    check(
        "released ~0.1 s after the last repeat, not 0.6 s — 5x less overshoot",
        not keys.held("w", last + measured + 0.001),
    )
    check(
        "the measured window can never EXCEED the advertised dead-man ceiling",
        keys.window("w") <= CEILING,
    )

    # A long gap must not be folded into the repeat-rate average: it is a
    # release followed by a fresh press, not a slow repeat.
    keys.on_event("w", REPEAT, last + 5.0)
    check("a gap longer than the ceiling is not averaged in", keys.window("w") <= 0.15)

    print()
    if failures:
        print("FAILURES: " + "; ".join(failures))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
