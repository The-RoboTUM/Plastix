"""servo_bus sign-magnitude handling, against a fake SDK packet handler (no port)."""
import pytest

from gripperx_arm import servo_bus as sb


class FakePkt:
    def __init__(self, raw):
        self.raw, self.written = raw, []

    def read2ByteTxRx(self, port, sid, addr):
        return self.raw, 0, 0

    def write2ByteTxRx(self, port, sid, addr, value):
        self.written.append((sid, addr, value))
        return 0, 0


class FakeScs:
    COMM_SUCCESS = 0


def bus(raw):
    b = object.__new__(sb.FeetechBus)
    b._scs, b._port, b._pkt = FakeScs(), None, FakePkt(raw)
    return b


@pytest.mark.parametrize('raw,value', [(0, 0), (4095, 4095), (32768 + 5, -5), (32768 + 300, -300)])
def test_present_position_is_decoded(raw, value):
    assert bus(raw).read_pos(1) == value


def test_goal_is_encoded_and_round_trips():
    b = bus(0)
    b.write_goal(2, -7)
    assert b._pkt.written == [(2, sb.ADDR_GOAL_POS, 32768 + 7)]
    assert sb.sm_decode(sb.sm_encode(-7, 15), 15) == -7
    with pytest.raises(ValueError):
        sb.sm_encode(1 << 15, 15)


class JunkPort:
    """A port that answers every read with junk bytes, forever: the SDK's header
    search discards them and loops without reaching its own reply timeout."""
    is_using = False
    tx_time_per_byte = 0.00001

    def clearPort(self):
        pass

    def writePort(self, packet):
        return len(packet)

    def readPort(self, length):
        return [0x00] * length

    def setPacketTimeout(self, packet_length):
        import time
        self.t0 = time.monotonic()

    def isPacketTimeout(self):
        import time
        return time.monotonic() - self.t0 > 0.04


def test_a_junk_stream_ends_the_transaction_within_the_bound():
    """2026-10-13 robot event: a bus read that never returns fills the executor. Each
    transaction is ended after TXN_MAX_S as a failed one, and the SDK's busy flag is
    reset so the next transaction runs."""
    import time
    scs = pytest.importorskip('scservo_sdk')
    b = object.__new__(sb.FeetechBus)
    b._scs, b._pkt, b._port = scs, scs.PacketHandler(0), JunkPort()
    b._bound()
    t0 = time.monotonic()
    assert b.read_pos(1) is None
    assert time.monotonic() - t0 < sb.TXN_MAX_S + 0.2
    assert b.timeouts == 1 and b._port.is_using is False
    assert b.write_goal(1, 5) is False and b.timeouts == 2


def test_stale_input_is_dropped_before_every_transaction():
    """Audit HG3 H3: a reply that arrived after its transaction was given up must not
    be read as the next same-id reply - the input buffer is reset at the start of
    every transaction, before the request goes out."""
    scs = pytest.importorskip('scservo_sdk')
    order = []

    class Ser:
        def reset_input_buffer(self):
            order.append('reset')

    class Port(JunkPort):
        ser = Ser()

        def writePort(self, packet):
            order.append('write')
            return len(packet)
    b = object.__new__(sb.FeetechBus)
    b._scs, b._pkt, b._port = scs, scs.PacketHandler(0), Port()
    b._bound()
    b.read_word(1, sb.ADDR_MIN_ANGLE_LIMIT)
    b.read_word(1, sb.ADDR_MAX_ANGLE_LIMIT)
    starts = [i for i, e in enumerate(order) if e == 'write']
    assert len(starts) == 2 and all(order[i - 1] == 'reset' for i in starts), order


# ------------------------------------------------------------------ audit EW1
def _bound_bus(pkt, ser=None):
    class Port:
        is_using = True

        def readPort(self, n):
            return []
    p = Port()
    if ser is not None:
        p.ser = ser
    b = object.__new__(sb.FeetechBus)
    b._scs, b._pkt, b._port = FakeScs(), pkt, p
    b._bound()
    return b


def test_an_unplugged_adapter_reaches_the_bus_guard():
    """H1: pyserial's SerialException (an OSError) must escape the bus so _BusGuard marks
    the bus lost and the node reopens it (2026-10-13 reconnect fix)."""
    serial = pytest.importorskip('serial')
    from gripperx_arm.arm_action_server import BusLost, _BusGuard

    class Pkt:
        def read2ByteTxRx(self, port, sid, addr):
            raise serial.SerialException('write failed: [Errno 5] Input/output error')

        def write1ByteTxRx(self, port, sid, addr, value):
            return self.read2ByteTxRx(port, sid, addr)
    b = _bound_bus(Pkt())
    lost = []
    g = _BusGuard(b, lambda e: lost.append(e))
    with pytest.raises(serial.SerialException):
        g.read_pos(1)
    assert g.lost is not None and len(lost) == 1 and b._port.is_using is False
    with pytest.raises(BusLost):
        g.read_pos(1)
    with pytest.raises(serial.SerialException):         # a torque-off is still attempted
        g.torque(1, False)
    assert len(lost) == 1


def test_termios_error_of_a_hung_up_tty_becomes_an_os_error():
    """M1: tcflush on a hung-up tty raises termios.error (no OSError); it must reach
    _BusGuard as an OSError - before the request and in the error handlers."""
    import termios
    from gripperx_arm.arm_action_server import _BusGuard

    class Ser:
        def reset_input_buffer(self):
            raise termios.error(5, 'Input/output error')

    class Pkt:
        def read2ByteTxRx(self, port, sid, addr):
            raise AssertionError('never reached: the reset fails first')
    b = _bound_bus(Pkt(), Ser())
    lost = []
    g = _BusGuard(b, lambda e: lost.append(e))
    with pytest.raises(OSError):
        g.read_word(1, sb.ADDR_PRESENT)
    assert g.lost is not None and len(lost) == 1

    class SdkTermios:
        def read2ByteTxRx(self, port, sid, addr):
            raise termios.error(5, 'Input/output error')
    b = _bound_bus(SdkTermios())
    with pytest.raises(OSError):
        b.read_word(1, sb.ADDR_PRESENT)
    assert b._port.is_using is False


def test_a_real_pty_hang_up_marks_the_bus_lost():
    import os
    import pty
    pytest.importorskip('scservo_sdk')
    from gripperx_arm.arm_action_server import _BusGuard
    m, s = pty.openpty()
    b = sb.FeetechBus(os.ttyname(s), 115200)
    b.open()
    lost = []
    g = _BusGuard(b, lambda e: lost.append(e))
    assert g.read_pos(1) is None                          # nobody answers: a failed read
    os.close(m)                                           # hang-up
    with pytest.raises(OSError):
        g.read_pos(1)
    assert g.lost is not None and len(lost) == 1
    os.close(s)


def test_type_error_is_a_programming_error_not_a_silent_none():
    class Pkt:
        def read2ByteTxRx(self, port, sid, addr):
            raise TypeError('bad call')
    b = _bound_bus(Pkt())
    with pytest.raises(TypeError):
        b.read_word(1, sb.ADDR_PRESENT)


def test_short_replies_and_timeouts_are_counted():
    class Pkt:
        def read2ByteTxRx(self, port, sid, addr):
            raise IndexError('list index out of range')
    b = _bound_bus(Pkt())
    assert b.read_word(1, sb.ADDR_PRESENT) is None and b.read_pos(1) is None
    assert b.health() == (0, 2)


def test_the_node_logs_a_growing_failure_count_rate_limited():
    import types
    from gripperx_arm.arm_action_server import ArmActionServer
    warns = []
    counts = [(0, 0)]
    stub = types.SimpleNamespace(
        BUS_HEALTH_LOG_PERIOD_S=30.0,
        get_logger=lambda: types.SimpleNamespace(warn=warns.append))
    guard = types.SimpleNamespace(raw=types.SimpleNamespace(health=lambda: counts[0]))
    log = ArmActionServer._log_bus_health
    log(stub, guard)
    assert warns == []                                    # nothing failed: silent
    counts[0] = (1, 3)
    log(stub, guard)
    assert len(warns) == 1 and '1 transaction(s)' in warns[0] and '3 by a malformed' in warns[0]
    counts[0] = (1, 9)
    log(stub, guard)
    assert len(warns) == 1                                # within the period: rate-limited
    stub._bus_health_logged = ((1, 3), -1e9)
    log(stub, guard)
    assert len(warns) == 2 and '+0 / +6' in warns[1]


def test_a_failed_read_says_why_and_a_good_one_clears_it():
    class Pkt:
        result = 7

        def read2ByteTxRx(self, port, sid, addr):
            return 0, self.result, 0

        def getTxRxResult(self, result):
            return '[TxRxResult] There is no status packet!'
    b = _bound_bus(Pkt())
    b._scs = FakeScs()
    assert b.read_pos(1) is None and 'no status packet' in b.last_error
    assert b.health() == (0, 0)                  # the SDK's own timeout is neither counter
    Pkt.result = 0
    assert b.read_pos(1) == 0 and b.last_error is None


def test_a_transaction_ended_by_the_bound_says_so():
    class Pkt:
        def read2ByteTxRx(self, port, sid, addr):
            raise sb.BusTimeout('x')
    b = _bound_bus(Pkt())
    assert b.read_word(1, sb.ADDR_PRESENT) is None and 'bound' in b.last_error
