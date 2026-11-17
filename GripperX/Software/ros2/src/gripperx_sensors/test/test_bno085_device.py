"""Bno085Device against a fake i2c device: what is sent to the sensor at start,
on recovery and on a sensor reset, and which quaternion reaches the output.

No ROS and no real time: the clock advances only through the injected sleep.
"""

import ast
import os
import struct
import sys

import pytest

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
sys.path.insert(0, _SRC)
from gripperx_sensors import bno08x_shtp as sh2  # noqa: E402
from gripperx_sensors.bno085_device import Bno085Device  # noqa: E402


def report(rid, status, *values, fmt="<hhh"):
    return bytes([rid, 0, status, 0]) + struct.pack(fmt, *values)


def packet(channel, payload):
    n = len(payload) + 4
    return bytes([n & 0xFF, n >> 8, channel, 0]) + payload


class FakeI2c:
    """Serves queued packets with the two-phase read; zero header when empty."""

    def __init__(self, packets=()):
        self.queue = list(packets)
        self.written = []
        self.opens = 0
        self.closes = 0

    def open(self):
        self.opens += 1

    def close(self):
        self.closes += 1

    def read(self, n):
        if not self.queue:
            return bytes(n)
        if n == 4 and len(self.queue[0]) > 4:
            return self.queue[0][:4]
        data = self.queue.pop(0)[:n]
        if len(data) > 4:
            # As the real sensor does on the re-read (measured 2026-10-20):
            # continuation bit set, sequence number advanced.
            data = bytes([data[0], data[1] | 0x80, data[2], (data[3] + 1) & 0xFF]) + data[4:]
        return data  # a bare 4-byte entry is a lone header

    def write(self, data):
        self.written.append((self.opens, bytes(data)))

    def set_features(self, open_no=None):
        """(report id, interval) of every Set Feature, optionally of one open."""
        out = []
        for n, w in self.written:
            if w[2] == sh2.CH_CONTROL and w[4] == sh2.CTRL_SET_FEATURE and open_no in (None, n):
                out.append((w[5], struct.unpack_from("<I", w, 9)[0]))
        return out

    def resets(self):
        return [n for n, w in self.written if w[2] == sh2.CH_EXECUTABLE and w[4:5] == b"\x01"]


class FakeTime:
    def __init__(self):
        self.t = 0.0

    def clock(self):
        return self.t

    def sleep(self, s):
        self.t += s
        return False


def make(bus, orientation_id=sh2.REPORT_GAME_ROTATION_VECTOR, **kw):
    ft = FakeTime()
    out = []
    dev = Bno085Device(bus, orientation_id=orientation_id, interval_us=20000,
                       clock=ft.clock, sleep=ft.sleep,
                       on_sample=lambda g, a, q: out.append((g, a, q)) or True, **kw)
    return dev, out, ft


def test_start_disables_every_managed_report_not_in_use():
    bus = FakeI2c()
    dev, _, _ = make(bus)
    dev.start(reset=False)
    sf = bus.set_features()
    assert sorted(r for r, i in sf if i == 0) == [0x05, 0x14]
    assert [(r, i) for r, i in sf if i] == [(0x01, 20000), (0x02, 20000), (0x08, 20000)]
    # the switch-offs come first, so nothing old is re-enabled after them
    assert all(i == 0 for _, i in sf[:2])


def test_rotation_vector_run_disables_the_game_rotation_vector():
    bus = FakeI2c()
    dev, _, _ = make(bus, orientation_id=sh2.REPORT_ROTATION_VECTOR)
    dev.start(reset=False)
    assert sorted(r for r, i in bus.set_features() if i == 0) == [0x08, 0x14]


def test_no_orientation_disables_both_rotation_vectors():
    bus = FakeI2c()
    dev, _, _ = make(bus, orientation_id=None)
    dev.start(reset=False)
    assert sorted(r for r, i in bus.set_features() if i == 0) == [0x05, 0x08, 0x14]


def test_every_report_the_query_tool_enables_is_managed():
    # A report bno085_query switches on but the driver does not know to switch
    # off would survive a killed query and stream into the driver's run.
    src = open(os.path.join(_SRC, "gripperx_sensors", "bno085_query.py")).read()
    names = {n.attr for n in ast.walk(ast.parse(src))
             if isinstance(n, ast.Attribute) and n.attr.startswith("REPORT_")}
    assert names
    assert {getattr(sh2, n) for n in names} <= sh2.MANAGED_REPORTS


def test_quaternion_only_from_the_configured_report():
    bus = FakeI2c()
    dev, out, _ = make(bus)
    accel = report(0x01, 3, 0, 0, 2511)
    gyro = report(0x02, 3, 0, 0, 0)
    rv_x = report(0x05, 3, 1 << 14, 0, 0, 0, 0, fmt="<hhhhh")      # (1,0,0,0)
    grv_w = report(0x08, 3, 0, 0, 0, 1 << 14, fmt="<hhhh")           # (0,0,0,1)
    # stale rotation vector only: no orientation yet, nothing published
    assert dev.handle(sh2.CH_REPORTS, accel + rv_x + gyro) is False
    assert out == []
    assert dev.handle(sh2.CH_REPORTS, grv_w + gyro) is True
    assert out[-1][2] == pytest.approx((0, 0, 0, 1))
    # a rotation vector after the game rotation vector must not overwrite it
    assert dev.handle(sh2.CH_REPORTS, grv_w + rv_x + gyro) is True
    assert out[-1][2] == pytest.approx((0, 0, 0, 1))
    assert dev.stats()["published"] == 2


def test_first_start_does_not_reset_but_recovery_does():
    bus = FakeI2c()
    dev, _, ft = make(bus, reset_on_start=False, stale_timeout=1.0)
    dev.run(keep_going=lambda: bus.opens < 2)
    # open 1: the first start, no soft reset; nothing arrives, so after the
    # stale timeout the driver restarts, and open 2 is a recovery with reset.
    assert bus.opens == 2
    assert bus.resets() == [2]
    assert dev.stats()["resets"] == 1


def test_reset_on_start_true_resets_the_first_start():
    bus = FakeI2c()
    dev, _, _ = make(bus, reset_on_start=True)
    dev.run(keep_going=lambda: bus.opens < 1)
    assert bus.resets() == [1]


def test_sensor_reset_reconfigures_including_switch_offs():
    bus = FakeI2c()
    dev, _, _ = make(bus)
    dev.handle(sh2.CH_EXECUTABLE, b"\x01")
    sf = bus.set_features()
    assert (0x05, 0) in sf and (0x08, 20000) in sf
    assert dev.stats()["device_resets"] == 1


def test_bus_busy_is_counted_apart_from_resets():
    class Busy(FakeI2c):
        def open(self):
            super().open()
            raise sh2.BusBusyError("held")

    bus = Busy()
    logs = []
    dev, _, _ = make(bus, log=lambda lvl, t: logs.append(lvl))
    dev.run(keep_going=lambda: bus.opens < 5)
    st = dev.stats()
    assert st["bus_busy"] == 5 and st["resets"] == 0 and st["io_errors"] == 0
    assert logs.count("warn") == 1  # 5 s of busy, logged once per 30 s
    assert bus.resets() == []  # still the first start: no reset once it frees


def test_fragments_dropped_appear_in_stats():
    frag = bytearray(packet(3, report(0x02, 0, 0, 0, 0)))
    frag[1] |= 0x80
    bus = FakeI2c([bytes(frag)])
    dev, _, _ = make(bus)
    assert dev.shtp.receive() is None
    assert dev.stats()["fragments_dropped"] == 1


def test_stop_interrupts_the_start_back_off():
    import threading
    stop = threading.Event()
    stop.set()
    bus = FakeI2c()
    dev = Bno085Device(bus, orientation_id=0x08, interval_us=20000, stop=stop)
    dev.start(reset=True)  # real stop.wait: returns at once instead of 0.5 s
    assert bus.set_features() == []  # left before configuring


class TimedI2c(FakeI2c):
    """Records the fake time of every soft reset; optionally a live sensor."""

    def __init__(self, ft):
        super().__init__()
        self.ft = ft
        self.reset_times = []
        self.alive = False

    def write(self, data):
        super().write(data)
        if data[2] == sh2.CH_EXECUTABLE and data[4:5] == b"\x01":
            self.reset_times.append(self.ft.t)

    def read(self, n):
        if self.alive and not self.queue:
            self.ft.t += 0.02  # one 50 Hz sample per poll
            self.queue.append(packet(3, report(0x01, 3, 0, 0, 2511)
                                     + report(0x08, 3, 0, 0, 0, 1 << 14, fmt="<hhhh")
                                     + report(0x02, 3, 0, 0, 0)))
        return super().read(n)


def make_timed(**kw):
    ft = FakeTime()
    bus = TimedI2c(ft)
    logs = []
    # idle_poll 0.1 s only keeps hours of simulated silence fast to run
    dev = Bno085Device(bus, orientation_id=0x08, interval_us=20000, clock=ft.clock,
                       sleep=ft.sleep, log=lambda lvl, t: logs.append((lvl, t)),
                       idle_poll=0.1, **kw)
    return dev, bus, ft, logs


def test_silent_sensor_resets_with_growing_back_off():
    dev, bus, ft, _ = make_timed(max_recovery_resets_per_hour=1000)
    dev.run(keep_going=lambda: ft.t < 30 * 60)
    gaps = [b - a for a, b in zip(bus.reset_times, bus.reset_times[1:])]
    assert len(bus.reset_times) <= 16  # ~3 s per cycle would be ~600
    assert all(b >= a for a, b in zip(gaps, gaps[1:]))
    assert gaps[-1] >= 300.0
    assert bus.opens > len(bus.reset_times)  # the first start had no reset


def test_silent_sensor_reset_count_is_capped_per_rolling_hour():
    dev, bus, ft, logs = make_timed(backoff_max_s=5.0)
    dev.run(keep_going=lambda: ft.t < 3 * 3600)
    for t in bus.reset_times:  # never more than 6 in any 1 h window
        assert sum(1 for u in bus.reset_times if t <= u < t + 3600) <= 6
    assert 6 < len(bus.reset_times) <= 18
    st = dev.stats()
    assert st["resets_suppressed"] > 0 and st["recovery_resets"] == len(bus.reset_times)
    assert bus.opens > 1000  # still reopening and polling without resets
    assert any(lvl == "error" for lvl, _ in logs)


def test_back_off_returns_to_initial_after_a_healthy_period():
    dev, bus, ft, _ = make_timed()
    state = {}

    def keep_going():
        if len(bus.reset_times) >= 4 and "alive_at" not in state:
            state["alive_at"] = ft.t
            state["grown"] = dev.backoff
            bus.alive = True
        return "alive_at" not in state or ft.t < state["alive_at"] + 90
    dev.run(keep_going=keep_going)
    assert state["grown"] >= 8.0
    assert dev.backoff == 1.0
    assert dev.stats()["published"] > 0


def test_protocol_error_in_start_up_drain_does_not_reset_on_first_start():
    # reset_on_start false: a garbled header while draining the previous run's
    # packets must not turn the first start into a recovery with a reset.
    bus = FakeI2c([b"\xff\xff\xff\xff"])
    dev, _, _ = make(bus, reset_on_start=False)
    dev.run(keep_going=lambda: bus.opens < 2 or not bus.set_features(open_no=2))
    assert dev.stats()["protocol_errors"] == 1
    assert bus.resets() == []
    assert bus.opens == 2 and (0x08, 20000) in bus.set_features(open_no=2)


# -- hung after a watchdog reset ---------------------------------------------

_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "bno085.yaml")


def _config():
    import yaml
    with open(_CONFIG) as f:
        return yaml.safe_load(f)["bno085_imu"]["ros__parameters"]


class ProductIdI2c(TimedI2c):
    """Answers the product id request with `cause`; samples only when alive."""

    def __init__(self, ft, cause):
        super().__init__(ft)
        self.cause = cause

    def write(self, data):
        super().write(data)
        if data[2] == sh2.CH_CONTROL and data[4] == sh2.CTRL_PRODUCT_ID_REQUEST:
            ident = (bytes([sh2.CTRL_PRODUCT_ID_RESPONSE, self.cause, 1, 2])
                     + (10004).to_bytes(4, "little") + (1000).to_bytes(4, "little")
                     + (7).to_bytes(2, "little"))
            self.queue.append(packet(sh2.CH_CONTROL, ident))


class FakeLine:
    def __init__(self, ft=None, bus=None, revive=False):
        self.pulses, self.bus, self.revive = 0, bus, revive

    def pulse(self):
        self.pulses += 1
        if self.revive:
            self.bus.alive = True


def make_hang(cause, with_line=False, revive=False):
    cfg = _config()
    ft = FakeTime()
    bus = ProductIdI2c(ft, cause)
    logs = []
    line = FakeLine(bus=bus, revive=revive) if with_line else None
    dev = Bno085Device(bus, orientation_id=0x08, interval_us=20000, clock=ft.clock,
                       sleep=ft.sleep, log=lambda lvl, t: logs.append((lvl, t)),
                       idle_poll=0.1, hung_recoveries=cfg["hung_recoveries"],
                       backoff_max_s=cfg["recovery_backoff_max_s"], reset_line=line)
    return dev, bus, ft, logs, line


def test_config_declares_hang_parameters_and_enabled_hw_reset():
    # HWR-42: RST wired 2026-11-09, hook enabled. Checks the active config is
    # complete (a partial set disables the hook) and that the settle time covers
    # the BNO08X datasheet ready minimum (1000-3927 v1.17 sec. 6.5.3:
    # t1 >= 90 ms + t2 ~4 ms; no INT is wired to wait on).
    cfg = _config()
    assert cfg["hung_recoveries"] >= 1
    assert cfg["hw_reset_gpio_line"] >= 0
    assert cfg["hw_reset_gpio_chip"].startswith("/dev/gpiochip")
    assert cfg["hw_reset_pulse_s"] > 0.0
    assert cfg["hw_reset_settle_s"] >= 0.094


def test_watchdog_cause_then_silent_latches_hung_with_operator_hint():
    dev, bus, ft, logs, _ = make_hang(cause=3)
    dev.run(keep_going=lambda: ft.t < 3600)
    assert dev.hung and dev.reset_cause == 3
    errs = [t for lvl, t in logs if lvl == "error" and "cut the IMU/Pi power" in t]
    assert 1 <= len(errs) <= 8  # rate-limited, not once per recovery
    assert dev.stats()["hung_episodes"] == 1
    assert dev.stats()["published"] == 0


def test_power_on_cause_silent_does_not_latch_hung():
    dev, bus, ft, logs, _ = make_hang(cause=1)
    dev.run(keep_going=lambda: ft.t < 3600)
    assert not dev.hung
    assert not any("cut the IMU" in t for _, t in logs)


def test_power_on_cause_healthy_publishes_and_stays_clear():
    dev, bus, ft, logs, _ = make_hang(cause=1)
    bus.alive = True
    dev.run(keep_going=lambda: ft.t < 5)
    assert dev.stats()["published"] > 50 and not dev.hung


def test_hung_clears_when_data_returns():
    dev, bus, ft, logs, _ = make_hang(cause=3)
    state = {}

    def keep_going():
        if dev.hung and "at" not in state:
            state["at"] = ft.t
            bus.alive = True
        return "at" not in state or ft.t < state["at"] + 700
    dev.run(keep_going=keep_going)
    assert "at" in state and not dev.hung and dev.stats()["published"] > 0
    assert any("hung state cleared" in t for _, t in logs)


def test_hw_reset_line_pulsed_once_before_latch_and_can_recover():
    dev, bus, ft, logs, line = make_hang(cause=3, with_line=True, revive=True)
    dev.run(keep_going=lambda: ft.t < 600)
    assert line.pulses == 1 and not dev.hung and dev.stats()["published"] > 0


def test_hw_reset_line_that_does_not_help_still_latches_after_one_pulse():
    dev, bus, ft, logs, line = make_hang(cause=3, with_line=True, revive=False)
    dev.run(keep_going=lambda: ft.t < 3600)
    assert line.pulses == 1 and dev.hung


def test_reset_line_factory_disabled_and_never_raises():
    from gripperx_sensors.bno085_reset_line import make_reset_line
    logs = []
    lg = lambda *a: logs.append(a)  # noqa: E731
    assert make_reset_line("", -1, 0.0, 0.0, lg) is None and not logs
    assert make_reset_line("/dev/x", 27, 0.0, 0.0, lg) is None and logs  # times unset


class _V1Line:
    def __init__(self, g):
        self.g = g

    def request(self, consumer, type, default_vals):
        if self.g.fail_request:
            raise OSError("busy")
        self.g.ev.append(("request", type, default_vals))

    def set_direction_input(self):
        self.g.ev.append(("input",))

    def release(self):
        self.g.ev.append(("release",))


class FakeGpiodV1:
    LINE_REQ_DIR_OUT = 3
    LINE_REQ_DIR_IN = 1

    def __init__(self, fail_request=False):
        self.ev = []
        self.fail_request = fail_request

    def Chip(self, path):  # noqa: N802
        g = self

        class C:
            def get_line(self, n):
                return _V1Line(g)

            def close(self):
                g.ev.append(("close",))
        return C()


def _ends_as_input(ev):
    return ev.index(("input",)) < ev.index(("release",))


def test_gpio_pulse_v1_drives_low_only_then_input_then_release():
    from gripperx_sensors.bno085_reset_line import GpioResetLine
    g = FakeGpiodV1()
    sl = []
    GpioResetLine("/dev/gpiochip4", 27, 0.25, 0.75, gpiod_module=g, sleep=sl.append).pulse()
    assert g.ev[0] == ("request", 3, [0])  # output, low, never high
    assert _ends_as_input(g.ev) and g.ev[-1] == ("close",)
    assert sl == [0.25, 0.75]


def test_gpio_pulse_v1_returns_to_input_when_interrupted_mid_pulse():
    from gripperx_sensors.bno085_reset_line import GpioResetLine
    g = FakeGpiodV1()

    def boom(_):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        GpioResetLine("/dev/gpiochip4", 27, 0.1, 0.1, gpiod_module=g, sleep=boom).pulse()
    assert _ends_as_input(g.ev) and g.ev[-1] == ("close",)


def test_gpio_pulse_v1_closes_chip_when_request_fails():
    from gripperx_sensors.bno085_reset_line import GpioResetLine
    g = FakeGpiodV1(fail_request=True)
    with pytest.raises(OSError):
        GpioResetLine("/dev/gpiochip4", 27, 0.1, 0.1, gpiod_module=g, sleep=lambda s: None).pulse()
    assert g.ev == [("close",)]


def _fake_v2(monkeypatch, ev):
    import types
    gl = types.ModuleType("gpiod.line")
    gl.Direction = types.SimpleNamespace(OUTPUT="out", INPUT="in")
    gl.Value = types.SimpleNamespace(INACTIVE="low", ACTIVE="high")
    pkg = types.ModuleType("gpiod")
    pkg.line = gl
    pkg.LineSettings = lambda direction, output_value=None: (direction, output_value)

    class Req:
        def reconfigure_lines(self, cfg):
            ev.append(("reconfigure", cfg))

        def release(self):
            ev.append("release")

    pkg.request_lines = lambda chip, consumer, config: ev.append(config) or Req()
    monkeypatch.setitem(sys.modules, "gpiod", pkg)
    monkeypatch.setitem(sys.modules, "gpiod.line", gl)
    return pkg


def test_gpio_pulse_v2_low_then_input_then_release(monkeypatch):
    from gripperx_sensors.bno085_reset_line import GpioResetLine
    ev = []
    pkg = _fake_v2(monkeypatch, ev)
    GpioResetLine("/dev/gpiochip4", 27, 0.1, 0.1, gpiod_module=pkg, sleep=lambda s: None).pulse()
    assert ev == [{27: ("out", "low")}, ("reconfigure", {27: ("in", None)}), "release"]


def test_gpio_pulse_v2_returns_to_input_when_interrupted(monkeypatch):
    from gripperx_sensors.bno085_reset_line import GpioResetLine
    ev = []
    pkg = _fake_v2(monkeypatch, ev)

    def boom(_):
        raise RuntimeError("x")

    with pytest.raises(RuntimeError):
        GpioResetLine("/dev/gpiochip4", 27, 0.1, 0.1, gpiod_module=pkg, sleep=boom).pulse()
    assert ev[1] == ("reconfigure", {27: ("in", None)}) and ev[-1] == "release"


def test_hw_reset_service_logic_disabled_and_enabled_and_serialized():
    dev, bus, ft, logs, line = make_hang(cause=1)
    ok, why = dev.hw_reset()
    assert not ok and "not enabled" in why
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
    seen = []
    line.pulse = lambda: seen.append(dev._io_lock.locked())
    assert dev.hw_reset() == (True, "pulsed") and seen == [True]  # held during the pulse
    line.pulse = lambda: 1 / 0
    ok, why = dev.hw_reset()
    assert not ok and "failed" in why


def test_commanded_hw_reset_reopens_plainly_without_soft_reset_or_backoff():
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
    dev.stale_timeout = 1.0
    bus.alive = True
    state = {}

    def pulse():  # the reset takes longer than stale_timeout_s; the chip then reports cause 5
        ft.t += 5.0
        bus.cause = 5
    line.pulse = pulse

    def keep_going():
        if ft.t > 2 and "at" not in state:
            state["at"], state["n"] = ft.t, dev.stats()["published"]
            assert dev.hw_reset() == (True, "pulsed")
        return "at" not in state or ft.t < state["at"] + 20
    dev.run(keep_going=keep_going)
    st = dev.stats()
    assert bus.reset_times == [] and st["recovery_resets"] == 0 and dev.backoff == 1.0
    assert dev.reset_cause == 5 and any("reset cause 5" in t for _, t in logs)
    assert st["published"] > state["n"] + 50
    assert not any("no gyroscope report" in t for _, t in logs)


def test_commanded_hw_reset_flag_is_set_before_the_lock_is_released():
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
    orig = dev._io_lock
    seen = {}

    class Spy:
        def __enter__(self):
            return orig.__enter__()

        def __exit__(self, *a):
            seen["pending_at_release"] = dev._hw_reset_pending.is_set()
            return orig.__exit__(*a)

        def locked(self):
            return orig.locked()
    dev._io_lock = Spy()
    assert dev.hw_reset()[0]
    assert seen["pending_at_release"]


# -- reboot evidence after a commanded hardware reset ------------------------

_BOOT_PACKETS = (  # what the chip sends after any reset (datasheet 1000-3927 sec. 5.2.1)
    # advertisement: TLVs incl. AppName (SH-2 SHTP Reference Manual 1000-3600 sec. 2.3)
    packet(sh2.CH_COMMAND, bytes([0x00, 1, 4, 1, 0, 0, 0, 8, 11]) + b"executable\x00"),
    packet(sh2.CH_EXECUTABLE, bytes([sh2.EXEC_RESET])),
    packet(sh2.CH_CONTROL, bytes([sh2.CTRL_COMMAND_RESPONSE, 0, sh2.CMD_INITIALIZE, 0, 0, 0])),
)


def _commanded_pulse_run(reaches_chip, boot=_BOOT_PACKETS, in_back_off=False):
    """Live sensor, one commanded pulse taking the configured pulse + settle
    time; if it reaches the chip, the chip's post-reset packets are queued."""
    cfg = _config()
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
    bus.alive = not in_back_off
    state = {}

    def pulse():
        ft.t += cfg["hw_reset_pulse_s"] + cfg["hw_reset_settle_s"]
        if reaches_chip:
            bus.queue[:] = list(boot)
        bus.alive = True
    line.pulse = pulse

    def fire():
        state["at"] = ft.t
        state["ret"] = dev.hw_reset()

    def sleep(s):  # in_back_off: the service call lands inside the recovery wait
        if in_back_off and "at" not in state and logs and "recovery start in" in logs[-1][1]:
            fire()
        return ft.sleep(s)
    dev.sleep = sleep

    def keep_going():
        if not in_back_off and ft.t > 2 and "at" not in state:
            fire()
        return "at" not in state or ft.t < state["at"] + 10
    dev.run(keep_going=keep_going)
    return dev, bus, logs, state


def _verdicts(logs):
    return [(lvl, t) for lvl, t in logs if t.startswith("commanded hardware reset")]


def test_commanded_hw_reset_confirmed_when_the_chip_sends_its_boot_packets():
    dev, bus, logs, state = _commanded_pulse_run(reaches_chip=True)
    hits = _verdicts(logs)
    assert len(hits) == 1 and hits[0][0] == "info"
    assert hits[0][1].startswith("commanded hardware reset CONFIRMED: chip rebooted (")
    for kind in ("advertisement on channel 0", "reset complete on channel 1",
                 "unsolicited initialize response on channel 2", "reset cause 1"):
        assert kind in hits[0][1]
    # expected after a commanded pulse: no device-reset warning, no budget used
    assert dev.stats()["device_resets"] == 0 and dev.stats()["recovery_resets"] == 0
    assert bus.reset_times == []


def test_commanded_hw_reset_confirmed_by_any_one_boot_packet():
    dev, bus, logs, state = _commanded_pulse_run(reaches_chip=True, boot=_BOOT_PACKETS[1:2])
    hits = _verdicts(logs)
    assert len(hits) == 1 and hits[0][0] == "info" and "reset complete on channel 1" in hits[0][1]


def test_commanded_hw_reset_not_confirmed_when_the_chip_keeps_streaming():
    dev, bus, logs, state = _commanded_pulse_run(reaches_chip=False)
    hits = _verdicts(logs)
    assert len(hits) == 1 and hits[0][0] == "warn"
    assert hits[0][1].startswith("commanded hardware reset NOT confirmed: no reboot evidence")
    assert bus.reset_times == [] and dev.stats()["recovery_resets"] == 0


def test_commanded_hw_reset_during_back_off_replaces_the_soft_reset():
    # Pulsed while a recovery waits: that start must not soft-reset (it would
    # bury the pulse's evidence) and must not spend the hourly reset budget.
    dev, bus, logs, state = _commanded_pulse_run(reaches_chip=True, in_back_off=True)
    assert bus.reset_times == [] and dev.stats()["recovery_resets"] == 0
    assert [lvl for lvl, _ in _verdicts(logs)] == ["info"]


def test_reader_waiting_on_the_pulse_leaves_the_boot_packets_to_the_re_open():
    # Real timing: the reader blocks on the bus lock while the service thread
    # pulses, then must not feed the first boot packet to handle().
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
    bus.alive = True
    orig = dev._io_lock
    state = {}

    class PulseWhileWaiting:
        def __enter__(self):
            if ft.t > 2 and "done" not in state:
                state["done"] = True
                bus.queue[:] = list(_BOOT_PACKETS)
                dev._hw_reset_pending.set()  # what hw_reset() leaves behind
            return orig.__enter__()

        def __exit__(self, *a):
            return orig.__exit__(*a)
    dev._io_lock = PulseWhileWaiting()
    dev.run(keep_going=lambda: "done" not in state or len(_verdicts(logs)) == 0)
    hits = _verdicts(logs)
    assert len(hits) == 1 and hits[0][0] == "info" and "advertisement on channel 0" in hits[0][1]
    assert dev.stats()["device_resets"] == 0


def test_hw_reset_service_waits_for_and_returns_the_verdict():
    import threading
    for reaches, word in ((True, "CONFIRMED"), (False, "NOT confirmed")):
        dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
        bus.alive = True
        line.pulse = (lambda: bus.queue.extend(_BOOT_PACKETS)) if reaches else (lambda: None)
        reader = threading.Thread(target=dev.run)
        reader.start()
        try:
            ok, msg = dev.hw_reset(wait_s=10.0)
        finally:
            dev.stop.set()
            reader.join(5.0)
        assert ok and msg.startswith("pulsed; commanded hardware reset " + word), msg


def test_channel_0_packet_without_advertisement_shape_is_no_evidence():
    assert Bno085Device.boot_packet_kind(sh2.CH_COMMAND, bytes([0x00, 0x00, 0x01, 0x00])) is None
    dev, bus, logs, state = _commanded_pulse_run(
        reaches_chip=True, boot=(packet(sh2.CH_COMMAND, bytes([0x00, 0x00, 0x01, 0x00])),))
    assert [lvl for lvl, _ in _verdicts(logs)] == ["warn"]


def test_failed_re_open_after_a_commanded_pulse_is_not_confirmed():
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)
    bus.alive = True
    real_open, state = bus.open, {}

    def open_():
        if state.get("fail"):
            state["fail"] = False
            raise OSError("bus gone")
        real_open()
    bus.open = open_

    def pulse():
        state["fail"] = True
    line.pulse = pulse

    def keep_going():
        if ft.t > 2 and "at" not in state:
            state["at"] = ft.t
            dev.hw_reset()
        return "at" not in state or ft.t < state["at"] + 10
    dev.run(keep_going=keep_going)
    hits = _verdicts(logs)
    assert hits and hits[0][0] == "warn" and "re-open after the pulse failed" in hits[0][1]
    assert dev._hw_reset_judged.is_set() and "bus gone" in dev.hw_reset_outcome


def test_threaded_pulse_overlapping_the_end_of_a_back_off_defers_the_recovery():
    # Audit finding: the service thread starts the pulse while the reader is
    # still in its recovery back-off; the reader wakes up mid-pulse. It must
    # neither spend reset budget nor soft-reset, and the verdict must hold.
    import threading
    import time as real_time
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)  # silent: recovery
    in_back_off, pulse_started = threading.Event(), threading.Event()

    def sleep(s):
        if not in_back_off.is_set() and logs and "recovery start in" in logs[-1][1]:
            in_back_off.set()
            assert pulse_started.wait(5.0)  # leave the back-off while the pulse runs
        return ft.sleep(s)
    dev.sleep = sleep

    def pulse():
        pulse_started.set()
        real_time.sleep(0.3)  # the reader is now out of its back-off, at the lock
        bus.queue[:] = list(_BOOT_PACKETS)
        bus.alive = True
    line.pulse = pulse

    reader = threading.Thread(target=dev.run)
    reader.start()
    try:
        assert in_back_off.wait(5.0)
        ok, msg = dev.hw_reset(wait_s=10.0)
    finally:
        dev.stop.set()
        reader.join(5.0)
    assert ok and msg.startswith("pulsed; commanded hardware reset CONFIRMED"), msg
    assert bus.reset_times == [] and dev.stats()["recovery_resets"] == 0


def test_hw_reset_service_without_a_verdict_says_so():
    dev, bus, ft, logs, line = make_hang(cause=1, with_line=True)  # no reader running
    ok, msg = dev.hw_reset(wait_s=0.01)
    assert ok and "no reboot verdict" in msg
