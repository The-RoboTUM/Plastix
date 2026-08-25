#!/usr/bin/env python3
"""GripperX clean-shutdown button daemon (HWR-40).

One button, two functions, measured by PRESS DURATION:

    press <  short_press_max_sec           -> Mode R  (restart stack, Pi stays up)
    short_press_max <= press < long_min    -> DISCARDED (dead zone, logged)
    press >= long_press_min_sec            -> Mode H  (stack down, Pi halts)

WHY THIS IS NOT A ROS NODE. The entire point of the button is to work when the
ROS2 stack is wedged, which is exactly when an rclpy node would be unable to
spin, unable to reach the DDS graph, or itself part of the problem. So: no
rclpy, no DDS, stdlib + libgpiod only, Restart=always, and short enough to audit
by reading. Do not "improve" this by making it a ROS node.

WHY NOT dtoverlay=gpio-shutdown. That overlay emits a single KEY_POWER event and
cannot measure how long the button was held, so it cannot distinguish the two
functions -- and it would claim the same GPIO line, so the two approaches are
mutually exclusive. This daemon supersedes it. Do not enable both.

ONE PATH, NOT TWO (SR-12 rev 3 / S2). This daemon decides *which mode*; it never
reimplements the stop sequence. Both modes invoke the same `stop_command` that
the operator CLI uses. A caller that reimplements the sequence is a defect.

libgpiod v1 API on purpose: the Pi ships python3-libgpiod 1.6.3, i.e.
`gpiod.Chip(path, OPEN_BY_PATH)` + `line.request(type=...)`, NOT the v2
`gpiod.request_lines()`. Same pattern as gripperx_control/lidar_power_node.py.
There are no gpiod CLI tools on the machine.

STATUS: NOT INSTALLED, NOT ENABLED, NOT EXECUTABLE. Nothing here is in effect.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import sys
import time

CONFIG_PATH = os.environ.get("GRIPPERX_BUTTON_CONFIG", "/etc/gripperx/gripperx-button.conf")

# Set by --dry-run. Recognise and log presses, never invoke the stop sequence.
# Deliberately a module global rather than a config key: a config key would
# persist into an installed unit, and a daemon that silently does nothing
# because someone left a flag set is a far worse failure than one that acts.
DRY_RUN = False

# Keys that MUST come from the config file. There are deliberately no built-in
# fallbacks for the safety-relevant ones: a missing or unreadable config must
# make the daemon idle loudly, never guess a threshold or an arming state.
REQUIRED_KEYS = (
    "gpio_chip_path",
    "gpio_line_offset",
    "short_press_max_sec",
    "long_press_min_sec",
    "stop_command",
    "mode_r_enabled",
    "mode_h_enabled",
)

DEFAULTS = {
    "gpio_consumer_name": "gripperx_button",
    "active_low": "true",
    "bias_pull_up": "true",
    "poll_interval_sec": "0.02",
    "debounce_sec": "0.02",
    "very_long_press_min_sec": "10.0",
    "very_long_press_action": "warn",
}

log = logging.getLogger("gripperx-button")


# --------------------------------------------------------------------------- #
#  config
# --------------------------------------------------------------------------- #
def parse_config(path: str) -> dict:
    """`key = value` lines, `#` comments. Hand-parsed: no shell, no eval, so a
    typo in this file can never execute anything."""
    values = dict(DEFAULTS)
    with open(path, "r", encoding="utf-8") as handle:
        for lineno, raw in enumerate(handle, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            if "=" not in line:
                raise ValueError(f"{path}:{lineno}: expected 'key = value', got {raw!r}")
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()

    missing = [key for key in REQUIRED_KEYS if key not in values]
    if missing:
        raise ValueError(f"{path}: missing required keys: {', '.join(missing)}")
    return values


def as_bool(values: dict, key: str) -> bool:
    raw = str(values[key]).strip().lower()
    if raw in ("true", "yes", "1", "on"):
        return True
    if raw in ("false", "no", "0", "off"):
        return False
    raise ValueError(f"{key}: expected a boolean, got {values[key]!r}")


class Config:
    def __init__(self, values: dict):
        self.chip_path = values["gpio_chip_path"]
        self.line_offset = int(values["gpio_line_offset"])
        self.consumer = values["gpio_consumer_name"]
        self.active_low = as_bool(values, "active_low")
        self.bias_pull_up = as_bool(values, "bias_pull_up")
        self.poll_interval = float(values["poll_interval_sec"])
        self.debounce = float(values["debounce_sec"])
        self.short_max = float(values["short_press_max_sec"])
        self.long_min = float(values["long_press_min_sec"])
        self.very_long_min = float(values["very_long_press_min_sec"])
        self.very_long_action = values["very_long_press_action"].strip().lower()
        self.stop_command = values["stop_command"].split()
        self.mode_r_enabled = as_bool(values, "mode_r_enabled")
        self.mode_h_enabled = as_bool(values, "mode_h_enabled")

        if not self.stop_command:
            raise ValueError("stop_command is empty")
        # The dead zone must be a real gap. Equal thresholds would silently turn
        # a marginal press into one of the two actions, which is the failure the
        # gap exists to prevent.
        if self.long_min <= self.short_max:
            raise ValueError(
                f"long_press_min_sec ({self.long_min}) must be strictly greater than "
                f"short_press_max_sec ({self.short_max}) -- the dead zone between them "
                "is a safety feature, not a rounding artefact"
            )
        if self.very_long_min < self.long_min:
            raise ValueError("very_long_press_min_sec must be >= long_press_min_sec")
        if self.very_long_action not in ("warn", "halt", "none"):
            raise ValueError(f"very_long_press_action: unknown value {self.very_long_action!r}")


# --------------------------------------------------------------------------- #
#  GPIO backend (libgpiod v1) -- all kernel specifics live in this class
# --------------------------------------------------------------------------- #
class GpiodLineInput:
    def __init__(self, chip_path: str, line_offset: int, consumer: str, bias_pull_up: bool):
        import gpiod  # imported here so the dependency stays inside the backend

        self._chip = gpiod.Chip(chip_path, gpiod.Chip.OPEN_BY_PATH)
        self._line = self._chip.get_line(line_offset)

        # The button module carries its own pull-up, so the bias flag is margin
        # rather than a requirement. Requested via getattr so an older libgpiod
        # that does not expose it degrades to "no bias" instead of crashing --
        # the external pull-up still defines the idle level.
        flags = 0
        if bias_pull_up:
            flag = getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_UP", None)
            if flag is None:
                log.warning(
                    "libgpiod has no LINE_REQ_FLAG_BIAS_PULL_UP; relying on the "
                    "module's external pull-up for the idle level"
                )
            else:
                flags = flag

        self._line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_IN, flags=flags)

    def get_value(self) -> int:
        return int(self._line.get_value())

    def release(self) -> None:
        if self._line is not None:
            self._line.release()
            self._line = None
        if self._chip is not None:
            self._chip.close()
            self._chip = None


# --------------------------------------------------------------------------- #
#  daemon
# --------------------------------------------------------------------------- #
class ButtonDaemon:
    def __init__(self, config: Config):
        self.cfg = config
        self._running = True
        self._gpio = GpiodLineInput(
            chip_path=config.chip_path,
            line_offset=config.line_offset,
            consumer=config.consumer,
            bias_pull_up=config.bias_pull_up,
        )
        log.info(
            "watching %s line %d (consumer=%s, active_low=%s) | "
            "short<%.1fs=Mode R%s | dead zone %.1f-%.1fs | long>=%.1fs=Mode H%s",
            config.chip_path, config.line_offset, config.consumer, config.active_low,
            config.short_max, "" if config.mode_r_enabled else " [DISARMED]",
            config.short_max, config.long_min,
            config.long_min, "" if config.mode_h_enabled else " [DISARMED]",
        )
        self._preflight()

    def _preflight(self) -> None:
        """Report up front whether the sequence we would call is actually there,
        so the failure is visible at start rather than at the moment someone
        presses the button in anger."""
        # The script is the LAST absolute path in the command, not the last
        # token: with a privilege wrapper the command reads
        # "/usr/bin/sudo -n /usr/local/bin/gripperx-stack-stop", and with a
        # trailing flag the last token would not be a path at all.
        paths = [tok for tok in self.cfg.stop_command if tok.startswith("/")]
        if not paths:
            log.error("stop_command contains no absolute path: %s",
                      " ".join(self.cfg.stop_command))
            return
        target = paths[-1]
        if not os.path.exists(target):
            log.error(
                "stop_command target %s does NOT EXIST -- every press will be "
                "recognised and then fail. The sequence script is not deployed.",
                target,
            )
        elif not os.access(target, os.X_OK):
            log.error("stop_command target %s exists but is not executable", target)
        else:
            log.info("stop_command target %s present and executable", target)

    def stop(self, *_args) -> None:
        self._running = False

    def _pressed(self, raw: int) -> bool:
        return raw == 0 if self.cfg.active_low else raw == 1

    def _invoke(self, mode: str) -> None:
        command = self.cfg.stop_command + [f"--mode={mode}"]
        if mode == "restart":
            # USER DECISION 2026-08-25: A PHYSICAL BUTTON PRESS IS the per-test approval
            # SR-1 requires for Mode R. Rationale as given by the user: "whoever presses
            # the button knows what they are doing."
            #
            # Without this flag gripperx-stack-stop REFUSES --mode=restart with exit 2,
            # so an armed short press would be recognised and then fail -- the exact
            # failure mode this interlock exists to prevent, one level up.
            #
            # SCOPE, stated so it is not widened by accident:
            #   * SHORT press only. --mode=stop and --mode=halt never needed it.
            #   * Added HERE, at the one call site with a human finger behind it, and
            #     NOT inside gripperx-stack-stop -- the operator CLI keeps its guard.
            #
            # STANDING ON: the restart path was measured motion-free TWICE on
            # 2026-08-25 -- the 11:33 cold boot (steering held at the limp angles,
            # `No command written on activation`) and a full contract restart ON BLOCKS
            # under per-test approval (/hw/steer_states byte-identical before/after).
            # Both remain CONDITIONAL on center_on_startup:false and on the interim
            # arm_home_on_startup:=false (#340). If either is lifted, revisit this.
            command += ["--approve-motion"]
            log.warning("Mode R: taking THIS BUTTON PRESS as the SR-1 per-test approval "
                        "(user decision 2026-08-25). Passing --approve-motion.")
        if DRY_RUN:
            # --dry-run exists so the WIRING and the THRESHOLDS can be verified on
            # the real machine without the daemon being installed and without any
            # stop, restart or halt happening. That is the whole point: it turns
            # the one test that would otherwise cost a boot cycle into a two-minute
            # observation that cannot affect the robot. It stops short of the
            # invocation and of nothing else -- debounce, timing and classification
            # are the same code that runs in earnest.
            log.info("DRY RUN -- would invoke: %s   (nothing executed)", " ".join(command))
            return
        log.info("invoking: %s", " ".join(command))
        try:
            result = subprocess.run(command, check=False)  # no shell, argv only
        except OSError as exc:
            log.error("could not invoke the stop sequence: %s", exc)
            return
        if result.returncode == 0:
            log.info("stop sequence (--mode=%s) completed", mode)
        else:
            log.error("stop sequence (--mode=%s) exited %d", mode, result.returncode)

    def _classify_and_act(self, duration: float) -> None:
        if duration < self.cfg.short_max:
            if not self.cfg.mode_r_enabled:
                # The whole point of the interlock: say precisely what was
                # recognised, what was NOT done, and why -- never fail silently.
                log.warning(
                    "SHORT press recognised (%.2f s) -> Mode R, but Mode R is "
                    "DISARMED (mode_r_enabled=false). NOTHING WAS DONE. The three "
                    "code/config conditions ARE deployed on this tree "
                    "(center_on_startup:=false, the D-SD1 on_deactivate fix, the "
                    "_on_timer steering hold) and a restart was measured on "
                    "2026-08-25 to move NOTHING -- but arming is a PROCESS "
                    "decision, not a code one: ExecStop is not wired, and an "
                    "unattended short press is a restart nobody approved (SR-1). "
                    "The motion-free result is also conditional on "
                    "center_on_startup:false and on the INTERIM "
                    "arm_home_on_startup:=false (issue #340). See "
                    "gripperx-button.conf.",
                    duration,
                )
                return
            log.info("SHORT press (%.2f s) -> Mode R (restart stack, Pi stays up)", duration)
            self._invoke("restart")
            return

        if duration < self.cfg.long_min:
            # The dead zone. Not an error, not an action -- a deliberate refusal.
            log.warning(
                "press of %.2f s falls in the DEAD ZONE (%.1f-%.1f s) and is "
                "DISCARDED. Nothing was done. Press briefly (<%.1f s) for a stack "
                "restart or hold (>=%.1f s) to shut the Pi down.",
                duration, self.cfg.short_max, self.cfg.long_min,
                self.cfg.short_max, self.cfg.long_min,
            )
            return

        if not self.cfg.mode_h_enabled:
            log.warning(
                "LONG press recognised (%.2f s) -> Mode H, but Mode H is DISARMED "
                "(mode_h_enabled=false). NOTHING WAS DONE.", duration,
            )
            return
        log.info("LONG press (%.2f s) -> Mode H (stack down, then Pi halts)", duration)
        self._invoke("halt")

    def _note_very_long(self, duration: float) -> None:
        """Called once, while the button is STILL HELD, past very_long_min.

        Shipped behaviour is "warn": log and do nothing. See the long-hold
        rationale in the module docstring's companion section below.
        """
        if self.cfg.very_long_action == "none":
            return
        if self.cfg.very_long_action == "warn":
            log.critical(
                "button held %.1f s (>= %.1f s). No escape action is armed. If the "
                "stack is unresponsive: release for the normal Mode H attempt; if "
                "that does not halt the Pi, use Q1 as the hard disconnect. "
                "(very_long_press_action=warn)",
                duration, self.cfg.very_long_min,
            )
            return
        # "halt": deliberately bypasses the ordered sequence. NOT the default and
        # NOT to be enabled without the user's explicit approval -- it halts with
        # commands still flowing and the drive stopping only by the ESP32's
        # CMD_TIMEOUT_MS backstop, which HWR-40 criterion 6 forbids as the
        # mechanism. It exists for the "nothing else responds" case only.
        log.critical(
            "button held %.1f s -> FORCED HALT, bypassing the ordered sequence "
            "(very_long_press_action=halt)", duration,
        )
        subprocess.run(["/usr/bin/systemctl", "poweroff", "--force"], check=False)

    def run(self) -> int:
        cfg = self.cfg
        press_start = None
        very_long_noted = False
        # Seed from the live level so a button already held at startup is not
        # mistaken for a fresh press.
        was_pressed = self._pressed(self._gpio.get_value())
        if was_pressed:
            log.warning("button already held at startup -- ignoring until released")

        while self._running:
            time.sleep(cfg.poll_interval)
            is_pressed = self._pressed(self._gpio.get_value())

            if is_pressed and not was_pressed:
                time.sleep(cfg.debounce)
                if not self._pressed(self._gpio.get_value()):
                    continue  # bounce or spike, not a press
                press_start = time.monotonic()
                very_long_noted = False

            elif is_pressed and press_start is not None and not very_long_noted:
                if time.monotonic() - press_start >= cfg.very_long_min:
                    very_long_noted = True
                    self._note_very_long(time.monotonic() - press_start)

            elif not is_pressed and was_pressed and press_start is not None:
                duration = time.monotonic() - press_start
                press_start = None
                self._classify_and_act(duration)

            was_pressed = is_pressed

        log.info("terminating, releasing GPIO line")
        self._gpio.release()
        return 0


def main() -> int:
    global DRY_RUN
    argv = sys.argv[1:]
    if "--dry-run" in argv:
        DRY_RUN = True
        argv.remove("--dry-run")
    if argv:
        print(
            "usage: gripperx-button-daemon.py [--dry-run]\n"
            "  --dry-run  recognise and log presses, never invoke the stop sequence.\n"
            "             For verifying wiring and thresholds on the real machine\n"
            "             without installing the daemon and without stopping anything.\n"
            "  config comes from $GRIPPERX_BUTTON_CONFIG (default %s)" % CONFIG_PATH,
            file=sys.stderr,
        )
        return 2

    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(message)s",  # systemd adds its own timestamp
        stream=sys.stdout,
    )
    try:
        config = Config(parse_config(CONFIG_PATH))
    except FileNotFoundError:
        log.error(
            "config %s not found -- idling without claiming the GPIO line. "
            "There are deliberately no built-in defaults for the thresholds or "
            "the arming flags: guessing either would be a safety decision.",
            CONFIG_PATH,
        )
        return 1
    except (ValueError, OSError) as exc:
        log.error("config %s rejected: %s", CONFIG_PATH, exc)
        return 1

    try:
        daemon = ButtonDaemon(config)
    except Exception as exc:  # GPIO claim failure: report it, let systemd retry
        log.error("could not claim %s line %d: %s", config.chip_path, config.line_offset, exc)
        return 1

    signal.signal(signal.SIGTERM, daemon.stop)
    signal.signal(signal.SIGINT, daemon.stop)
    return daemon.run()


if __name__ == "__main__":
    sys.exit(main())
