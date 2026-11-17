"""Optional hardware reset line (BNO085 RST, active low) behind a tiny interface.

The driver only needs `pulse()`. Per HWR-42 the line is driven LOW only for the
pulse and otherwise released (input, high-Z); it is never driven high. libgpiod
is imported lazily (v1 API as on the Pi, python3-gpiod 1.6.3; v2 also handled),
so nothing here is required on a machine without it. Chip, line, pulse length
and post-reset wait all come from parameters. On the robot the line is GPIO27
(header pin 13) and the values are set in config/bno085.yaml (decided by the
user 2026-11-09, OP-H16 Q2). The code defaults stay disabled: a line < 0, or a
time <= 0, keeps the hook off.
"""

from __future__ import annotations

import time

_CONSUMER = "bno085_reset"


class GpioResetLine:
    def __init__(self, chip: str, line: int, pulse_s: float, settle_s: float,
                 gpiod_module=None, sleep=time.sleep) -> None:
        if gpiod_module is None:
            import gpiod as gpiod_module
        self._g = gpiod_module
        self._v2 = hasattr(gpiod_module, "request_lines")
        self.chip, self.line = chip, line
        self.pulse_s, self.settle_s = pulse_s, settle_s
        self._sleep = sleep

    def pulse(self) -> None:
        """Low for pulse_s, then explicitly input again (release() alone does not
        reliably restore the direction on the RP1), then wait settle_s. The
        switch back to input runs on every exit path."""
        g = self._g
        if self._v2:
            from gpiod.line import Direction, Value
            req = g.request_lines(self.chip, consumer=_CONSUMER, config={
                self.line: g.LineSettings(direction=Direction.OUTPUT,
                                          output_value=Value.INACTIVE)})
            try:
                self._sleep(self.pulse_s)
            finally:
                try:
                    req.reconfigure_lines({self.line: g.LineSettings(direction=Direction.INPUT)})
                finally:
                    req.release()
        else:
            chip = g.Chip(self.chip)
            try:
                line = chip.get_line(self.line)
                line.request(consumer=_CONSUMER, type=g.LINE_REQ_DIR_OUT, default_vals=[0])
                try:
                    self._sleep(self.pulse_s)
                finally:
                    try:
                        line.set_direction_input()
                    finally:
                        line.release()
            finally:
                chip.close()
        self._sleep(self.settle_s)


def make_reset_line(chip: str, line: int, pulse_s: float, settle_s: float, log,
                    sleep=time.sleep):
    """None when disabled or unusable (logged), never raises. `sleep` should return early on shutdown."""
    if line < 0:
        return None
    if not chip or pulse_s <= 0.0 or settle_s <= 0.0:
        log("error", "hw_reset_gpio_line set but chip, hw_reset_pulse_s or "
                     "hw_reset_settle_s missing: hardware reset disabled")
        return None
    try:
        import gpiod  # noqa: F401
        return GpioResetLine(chip, line, pulse_s, settle_s, sleep=sleep)
    except Exception as exc:  # noqa: BLE001 - gpiod missing
        log("error", f"hardware reset line {chip}:{line} unavailable ({exc!r}); disabled")
        return None
