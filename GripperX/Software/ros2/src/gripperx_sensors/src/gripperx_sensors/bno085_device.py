"""BNO085 bring-up, report configuration, packet handling and recovery. No ROS.

The ROS node (bno085_node) owns one Bno085Device and runs `run()` in a reader
thread; everything that decides what is sent to the sensor lives here, so it can
be tested against a fake bus without a ROS context.

REPORT CONFIGURATION. Set Feature is volatile across a sensor reset but not
across a host process: a report a previous run (or a killed `bno085_query
--compare-raw`) switched on keeps streaming. Every configuration therefore
first switches off (report interval 0, SH-2 Reference Manual 1000-3625 rev 1.9
section 5.4.1) each report in MANAGED_REPORTS this run does not use, then
enables the ones it does. In addition the quaternion is only ever taken from
the configured orientation report id, so reports still in flight from the old
configuration cannot mix into the output.

RECOVERY RESETS ARE RATE-LIMITED. Every non-power-up reset persists the RAM
dynamic calibration data to flash (BNO08X datasheet 1000-3927 v1.17 section
3.4), and the flash's write endurance is not stated there (TO-VERIFY). A sensor
that stays silent must therefore not be reset every few seconds: recovery
starts wait an exponentially growing back-off, and after
`max_recovery_resets_per_hour` resets within a rolling hour the driver keeps
reopening and polling the bus WITHOUT a reset (counted as resets_suppressed,
logged as an error). The back-off returns to its initial value once samples
have flowed for `healthy_period_s`. The numbers are design choices (see
config/bno085.yaml), not datasheet values.

HUNG AFTER A WATCHDOG RESET. Measured 2026-11-02: after a sensor-side watchdog
reset (SH-2 product id reset cause 3) the chip answered on I2C but never again
delivered a gyroscope report, through 7 hours of software recoveries; a real
power-off cleared it (cause 1). Once cause 3 has been seen, `hung_recoveries`
consecutive recoveries without a single gyroscope report latch `hung` (an
operator hint at error level, repeated at most every `_HUNG_LOG_PERIOD_S`);
recovery keeps running in the background and the first sample clears the latch.
An optional hardware reset line (`reset_line`, default none) is pulsed once per
episode just before the latch is set. The cause is remembered across recoveries
on purpose: our own soft reset replaces it in the next product id.

PROOF THAT A COMMANDED PULSE REBOOTED THE CHIP. The product id's reset cause
cannot tell: the SH-2 Reference Manual 1000-3625 rev 1.9 section 6.3.2 lists
"4 - External Reset" but does not say what an nRST pulse reports, and on the
robot (2026-11-09) the re-open after a pulse read cause 1 as before it. What
does tell is what the chip sends after any reset (BNO08X datasheet 1000-3927
v1.17 section 5.2.1): the SHTP advertisement on channel 0, "reset complete" on
channel 1 and the unsolicited Initialize response (0x84) on channel 2. This
driver never requests any of them, and the reader drains the bus continuously
up to the pulse, so seeing one in the re-open after a commanded pulse is the
evidence; seeing none, or a re-open that fails, is logged as NOT confirmed. The
~/hw_reset response waits up to HW_RESET_VERDICT_WAIT_S (3 s, fixed here, not a
parameter) for that verdict. Limitation: a reboot of the chip's own (watchdog,
brown-out) while the bus was closed just before the pulse also queues these
packets and would read as CONFIRMED; the reader closes the bus only during a
recovery's 0.2 s pause and back-off, so this needs a fault in that window.
"""

from __future__ import annotations

import collections
import threading
import time
from typing import Callable

from gripperx_sensors import bno08x_shtp as sh2

_BUSY_LOG_PERIOD_S = 30.0
_HUNG_LOG_PERIOD_S = 600.0
HW_RESET_VERDICT_WAIT_S = 3.0  # service wait for the re-open (start() takes ~1 s at most)
RESET_CAUSE_WATCHDOG = 3  # SH-2 product id response, byte 1 (reset cause)
HUNG_HINT = ("BNO085 stuck after watchdog reset: software reset does not recover it, "
             "cut the IMU/Pi power")


def sh2_cause_name(cause: int | None) -> str:
    return "unknown" if cause is None else str(cause)


class Bno085Device:
    def __init__(self, bus, *, orientation_id: int | None, interval_us: int,
                 reset_on_start: bool = False, max_errors: int = 20,
                 stale_timeout: float = 1.0, idle_poll: float = 0.002,
                 on_sample: Callable | None = None, log: Callable | None = None,
                 stop: threading.Event | None = None,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], bool] | None = None,
                 backoff_initial_s: float = 1.0, backoff_max_s: float = 300.0,
                 max_recovery_resets_per_hour: int = 6,
                 healthy_period_s: float = 60.0,
                 hung_recoveries: int = 3, reset_line=None) -> None:
        """on_sample(gyro, accel, quat_xyzw_or_None) -> bool (False: not
        published, e.g. shutting down). log(level, text) with level one of
        debug/info/warn/error/fatal. sleep(s) -> True when a stop was requested;
        the default waits on `stop`, so destroy does not wait out a back-off.
        reset_line: optional object with pulse() -> None (hardware RST)."""
        self.bus = bus
        self.shtp = sh2.Shtp(bus)
        self.orientation_id = orientation_id
        self.interval_us = interval_us
        self.reset_on_start = reset_on_start
        self.max_errors = max_errors
        self.stale_timeout = stale_timeout
        self.idle_poll = idle_poll
        self.on_sample = on_sample or (lambda gyro, accel, quat: True)
        self.log = log or (lambda level, text: None)
        self.stop = stop or threading.Event()
        self.clock = clock
        self.sleep = sleep or self.stop.wait
        self.backoff_initial = backoff_initial_s
        self.backoff_max = backoff_max_s
        self.max_resets_per_hour = max_recovery_resets_per_hour
        self.healthy_period = healthy_period_s
        self.backoff = backoff_initial_s
        self._reset_times = collections.deque()
        self._last_suppress_log = None
        self.hung_recoveries = hung_recoveries
        self.reset_line = reset_line
        self._hw_reset_pending = threading.Event()  # commanded pulse: re-open, no soft reset
        self._hw_reset_judged = threading.Event()  # reboot verdict of the last commanded pulse
        self.hw_reset_outcome = ""
        self.boot_evidence: list[str] = []
        self._io_lock = threading.Lock()  # bus transactions vs. a hardware reset
        self.reset_cause = None
        self.hung = False
        self._watchdog_seen = False
        self._silent_recoveries = 0
        self._hw_reset_tried = False
        self._got_sample = False
        self._last_hung_log = None
        self.accel = None
        self.quat = None
        self.status = {"accel": 0, "gyro": 0, "quat": 0}
        self._stats_lock = threading.Lock()
        self._stats = dict(published=0, protocol_errors=0, io_errors=0, unknown_reports=0,
                           resets=0, device_resets=0, bus_busy=0, lock_denied=0,
                           recovery_resets=0, resets_suppressed=0, hung_episodes=0)
        self._last_busy_log = None

    # -- counters -----------------------------------------------------------

    def bump(self, key: str, n: int = 1) -> None:
        with self._stats_lock:
            self._stats[key] += n

    def stats(self) -> dict:
        with self._stats_lock:
            out = dict(self._stats)
        out["fragments_dropped"] = self.shtp.fragments_dropped
        return out

    # -- configuration ------------------------------------------------------

    def used_reports(self) -> list[int]:
        ids = [sh2.REPORT_ACCELEROMETER, sh2.REPORT_GYROSCOPE]
        if self.orientation_id is not None:
            ids.append(self.orientation_id)
        return ids

    def configure_reports(self) -> None:
        used = self.used_reports()
        for rid in sorted(sh2.MANAGED_REPORTS - set(used)):
            self.shtp.send(sh2.CH_CONTROL, sh2.set_feature_payload(rid, 0))
            self.sleep(0.01)
        for rid in used:
            self.shtp.send(sh2.CH_CONTROL, sh2.set_feature_payload(rid, self.interval_us))
            self.sleep(0.01)

    def start(self, reset: bool, after_hw_reset: bool = False) -> None:
        """Open the bus, optionally soft-reset the sensor, read its identity,
        configure reports. Packets that only a rebooted chip sends are collected
        in `boot_evidence`; after a commanded pulse they are expected, not a fault."""
        self.boot_evidence = []
        self.bus.open()
        if reset:
            self.shtp.send(sh2.CH_EXECUTABLE, bytes([sh2.EXEC_RESET]))
            if self.sleep(0.5):
                return
        # After a reset: advertisement, reset-complete, unsolicited init
        # response. Without one: reports still streaming from a previous run.
        # Neither is wanted; the reports are reconfigured below in any case.
        for _ in range(64):
            pkt = self.shtp.receive()
            if pkt is None:
                break
            self._note_boot_packet(*pkt)
        self.shtp.send(sh2.CH_CONTROL, bytes([sh2.CTRL_PRODUCT_ID_REQUEST, 0]))
        deadline = self.clock() + 1.0
        ident = None
        while ident is None and self.clock() < deadline:
            pkt = self.shtp.receive()
            if pkt is None:
                if self.sleep(0.01):
                    return
                continue
            ch, payload = pkt
            self._note_boot_packet(ch, payload)
            if ch == sh2.CH_CONTROL and payload and payload[0] == sh2.CTRL_PRODUCT_ID_RESPONSE:
                ident = payload
            elif not reset and not after_hw_reset and self._is_reset_indication(ch, payload):
                # Not passed to handle(): it would only re-send the
                # configuration that configure_reports() below sends anyway.
                # Other packets here are pre-configuration samples; dropping
                # up to 1 s of them at start-up loses nothing.
                self.bump("device_resets")
                self.log("warn", "BNO085 reported a reset during start-up")
        if ident is not None and len(ident) >= 14:
            part = int.from_bytes(ident[4:8], "little")
            build = int.from_bytes(ident[8:12], "little")
            patch = int.from_bytes(ident[12:14], "little")
            self.reset_cause = ident[1]
            if self.reset_cause == RESET_CAUSE_WATCHDOG:
                self._watchdog_seen = True
            self.log("info", f"SH-2 product id: part {part}, firmware {ident[2]}.{ident[3]}.{patch} "
                             f"build {build}, reset cause {ident[1]}")
        else:
            self.reset_cause = None
            self.log("warn", "no SH-2 product id response within 1 s; continuing")
        self.configure_reports()

    def restart(self, why: str) -> None:
        self.bump("resets")
        self.log("warn", f"BNO085 restart: {why}")
        self.accel = None
        self.quat = None
        try:
            self.bus.close()
        except OSError:
            pass
        self.sleep(0.2)

    # -- packets ------------------------------------------------------------

    @staticmethod
    def boot_packet_kind(channel: int, payload: bytes) -> str | None:
        """What a chip sends only after it has (re)booted (module docstring)."""
        if channel == sh2.CH_COMMAND and (b"executable" in payload or b"sensorhub" in payload):
            # AppName values of the advertisement, SH-2 SHTP Reference Manual
            # 1000-3600 rev 1.6 section 2.3, Figure 3.
            return "SHTP advertisement on channel 0"
        if channel == sh2.CH_EXECUTABLE and payload[:1] == bytes([sh2.EXEC_RESET]):
            return "reset complete on channel 1"
        if (channel == sh2.CH_CONTROL and payload[:1] == bytes([sh2.CTRL_COMMAND_RESPONSE])
                and len(payload) > 2 and payload[2] == sh2.CMD_INITIALIZE):
            return "unsolicited initialize response on channel 2"
        return None

    def _note_boot_packet(self, channel: int, payload: bytes) -> None:
        kind = self.boot_packet_kind(channel, payload)
        if kind is not None and kind not in self.boot_evidence:
            self.boot_evidence.append(kind)

    def _judge_hw_reset(self, failed: Exception | None = None) -> None:
        """Log whether the re-open after a commanded pulse saw the chip reboot."""
        cause = sh2_cause_name(self.reset_cause)
        if failed is not None:
            self.hw_reset_outcome = (f"commanded hardware reset NOT confirmed: re-open after "
                                     f"the pulse failed ({failed!r})")
            self.log("warn", self.hw_reset_outcome)
        elif self.boot_evidence:
            self.hw_reset_outcome = (f"commanded hardware reset CONFIRMED: chip rebooted "
                                     f"({', '.join(self.boot_evidence)}; reset cause {cause})")
            self.log("info", self.hw_reset_outcome)
        else:
            self.hw_reset_outcome = ("commanded hardware reset NOT confirmed: no reboot evidence "
                                     "(no advertisement, reset complete or unsolicited "
                                     "initialize response) in the re-open after the pulse; "
                                     f"reset cause {cause}")
            self.log("warn", self.hw_reset_outcome)
        self._hw_reset_judged.set()

    @staticmethod
    def _is_reset_indication(channel: int, payload: bytes) -> bool:
        if channel == sh2.CH_EXECUTABLE and payload[:1] == bytes([sh2.EXEC_RESET]):
            return True
        return (channel == sh2.CH_CONTROL and payload[:1] == bytes([sh2.CTRL_COMMAND_RESPONSE])
                and len(payload) > 2 and payload[2] == sh2.CMD_INITIALIZE)

    def handle(self, channel: int, payload: bytes) -> bool:
        """Return True when a sample was published."""
        if self._is_reset_indication(channel, payload):
            # The sensor reset itself (brown-out, watchdog): its features are gone.
            self.bump("device_resets")
            self.log("warn", "BNO085 reported a reset; re-configuring reports")
            self.accel = None
            self.quat = None
            self.configure_reports()
            return False
        if channel not in (sh2.CH_REPORTS, sh2.CH_WAKE_REPORTS):
            return False
        s = sh2.parse_input_packet(payload)
        if s.unknown_report_ids:
            self.bump("unknown_reports", len(s.unknown_report_ids))
        if s.accel is not None:
            self.accel = s.accel
            self.status["accel"] = s.accel_status
        q = s.quats.get(self.orientation_id) if self.orientation_id is not None else None
        if q is not None:
            self.quat = q.xyzw
            self.status["quat"] = q.status
        if s.gyro is None:
            return False
        self.status["gyro"] = s.gyro_status
        if self.accel is None or (self.orientation_id is not None and self.quat is None):
            return False
        if not self.on_sample(s.gyro, self.accel, self.quat):
            return False
        self.bump("published")
        return True

    # -- hang detection -----------------------------------------------------

    def _note_silent_recovery(self) -> None:
        """A start (recovery or first) ended without any gyroscope report."""
        if not self._watchdog_seen:
            return
        self._silent_recoveries += 1
        if self._silent_recoveries < self.hung_recoveries:
            return
        if self.reset_line is not None and not self._hw_reset_tried:
            self._hw_reset_tried = True
            self._silent_recoveries = self.hung_recoveries - 1  # one more try after the pulse
            ok, why = self.hw_reset(commanded=False)
            self.log("warn" if ok else "error", f"BNO085 hardware reset: {why}")
            return
        now = self.clock()
        if not self.hung:
            self.bump("hung_episodes")
        self.hung = True
        if self._last_hung_log is None or now - self._last_hung_log >= _HUNG_LOG_PERIOD_S:
            self._last_hung_log = now
            self.log("error", f"{HUNG_HINT} ({self._silent_recoveries} recoveries without "
                              f"a gyroscope report, reset cause {sh2_cause_name(self.reset_cause)})")

    def hw_reset(self, commanded: bool = True, wait_s: float = 0.0) -> tuple[bool, str]:
        """One hardware reset pulse, serialized with bus transactions (any thread).
        A commanded pulse (the service) is followed by a plain re-open that reads
        and logs the product id (reset cause) and the reboot verdict without a
        soft reset, a recovery count or a back-off; the pulse is a deliberate act,
        not a fault. Pulsed during a recovery back-off, that recovery's start
        becomes the plain re-open. wait_s > 0 (never from the reader thread):
        wait that long for the verdict and return it as the message."""
        if self.reset_line is None:
            return False, "hardware reset line not enabled (hw_reset_gpio_* parameters)"
        with self._io_lock:
            if commanded:
                self._hw_reset_judged.clear()
            try:
                self.reset_line.pulse()
            except Exception as exc:  # noqa: BLE001 - a broken GPIO must not kill the reader
                return False, f"failed: {exc!r}"
            if commanded:  # inside the lock: the reader cannot see the aged timer first
                self._hw_reset_pending.set()
        if commanded and wait_s > 0.0:
            if self._hw_reset_judged.wait(wait_s):
                return True, f"pulsed; {self.hw_reset_outcome}"
            return True, f"pulsed; no reboot verdict within {wait_s:.0f} s, see the node log"
        return True, "pulsed"

    def _note_data(self) -> None:
        if self.hung:
            self.log("info", "BNO085 delivers data again; hung state cleared")
        self.hung = False
        self._watchdog_seen = False
        self._silent_recoveries = 0
        self._hw_reset_tried = False
        self._last_hung_log = None

    # -- main loop ----------------------------------------------------------

    def _busy_log_due(self) -> bool:
        now = self.clock()
        if self._last_busy_log is None or now - self._last_busy_log >= _BUSY_LOG_PERIOD_S:
            self._last_busy_log = now
            return True
        return False

    def _recovery_reset_allowed(self) -> bool:
        """Wait the back-off, then decide whether this recovery may reset."""
        now = self.clock()
        while self._reset_times and now - self._reset_times[0] >= 3600.0:
            self._reset_times.popleft()
        if len(self._reset_times) >= self.max_resets_per_hour:
            self.bump("resets_suppressed")
            if self._last_suppress_log is None or now - self._last_suppress_log >= 600.0:
                self._last_suppress_log = now
                self.log("error", f"BNO085 still silent after {len(self._reset_times)} recovery "
                                  "resets within an hour; reopening without a reset (each "
                                  "reset writes calibration to flash)")
            return False
        self._reset_times.append(now)
        self.bump("recovery_resets")
        return True

    def run(self, keep_going: Callable[[], bool] = lambda: True) -> None:
        started = False
        first_start = True
        errors = 0
        last_sample = self.clock()
        started_at = last_sample
        plain_restart = False
        while not self.stop.is_set() and keep_going():
            try:
                if self._hw_reset_pending.is_set():
                    self._hw_reset_pending.clear()
                    started, plain_restart = False, True
                    self.accel = self.quat = None
                    try:
                        self.bus.close()
                    except OSError:
                        pass
                if not started:
                    # Only the very first start may skip the reset; every
                    # later start is a recovery from an unknown sensor state,
                    # rate-limited (module docstring).
                    reset = self.reset_on_start
                    recovering = not first_start and not plain_restart
                    if recovering:
                        self.log("info", f"BNO085 recovery start in {self.backoff:.0f} s")
                        if self.sleep(self.backoff):
                            break
                        self.backoff = min(self.backoff * 2.0, self.backoff_max)
                    with self._io_lock:
                        # Decided under the lock a commanded pulse holds from
                        # before the pulse until its flag is set: this start
                        # either runs wholly before the pulse or sees the flag
                        # and becomes the plain re-open (a soft reset would
                        # bury the pulse's reboot evidence and spend budget).
                        if self._hw_reset_pending.is_set():
                            self._hw_reset_pending.clear()
                            plain_restart = True
                        elif recovering:
                            reset = self._recovery_reset_allowed()
                        after_hw_reset = plain_restart
                        if plain_restart:
                            plain_restart = False
                            reset = False
                            self.log("info", "BNO085 re-opening after a commanded hardware reset")
                        try:
                            self.start(reset=reset, after_hw_reset=after_hw_reset)
                        except Exception as exc:
                            if after_hw_reset:
                                self._judge_hw_reset(failed=exc)
                            raise
                    if self.stop.is_set():
                        break
                    if after_hw_reset:
                        self._judge_hw_reset()
                    first_start = False
                    started = True
                    errors = 0
                    self._got_sample = False
                    last_sample = started_at = self.clock()
                with self._io_lock:
                    if self._hw_reset_pending.is_set():
                        # Waited out a pulse on this lock: the next packet is
                        # reboot evidence for the re-open, not for handle().
                        continue
                    pkt = self.shtp.receive()
                    handled = pkt is not None and self.handle(*pkt)
                if pkt is None:
                    if (self.clock() - last_sample > self.stale_timeout
                            and not self._hw_reset_pending.is_set()):
                        started = False
                        if not self._got_sample:
                            self._note_silent_recovery()
                        self.restart(f"no gyroscope report for {self.stale_timeout:.1f} s")
                        continue
                    self.sleep(self.idle_poll)
                    continue
                errors = 0
                if handled:
                    last_sample = self.clock()
                    if not self._got_sample or self.hung:
                        self._got_sample = True
                        self._note_data()
                    if (self.backoff != self.backoff_initial
                            and last_sample - started_at >= self.healthy_period):
                        self.backoff = self.backoff_initial
            except sh2.BusBusyError as exc:
                # Another driver instance holds the sensor: not a fault of this
                # one, so neither a reset nor an I/O error.
                self.bump("bus_busy")
                if self._busy_log_due():
                    self.log("warn", f"{exc}; waiting for it")
                self.sleep(1.0)
            except sh2.LockPermissionError as exc:
                self.bump("lock_denied")
                if self._busy_log_due():
                    self.log("error", str(exc))
                self.sleep(5.0)
            except sh2.ShtpError as exc:
                self.bump("protocol_errors")
                errors += 1
                if errors >= self.max_errors:
                    started = False
                    self.restart(f"{errors} consecutive protocol errors, last: {exc}")
                    errors = 0
            except OSError as exc:
                self.bump("io_errors")
                errors += 1
                if errors >= self.max_errors or not started:
                    started = False
                    self.restart(f"I/O error ({errors} consecutive): {exc}")
                    errors = 0
                    self.sleep(1.0)
                else:
                    self.sleep(0.005)
