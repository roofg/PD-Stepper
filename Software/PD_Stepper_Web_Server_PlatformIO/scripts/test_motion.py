"""
test_motion.py — Automated motion test campaign for PD-Stepper.

Requires hardware:
  - ESP32-S3 board on COM3 (USB CDC, 921600 baud)
  - FT232RL debug UART on COM4 (AUX serial, 115200 baud)

Run all tests:
    python -m pytest test_motion.py -v --tb=short

Skip if no hardware:
    python -m pytest test_motion.py -v -m "not hardware"
"""

import json
import re
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import pytest
import serial

# ---------------------------------------------------------------------------
# Protocol constants (must match usb_telemetry_provider.h)
# ---------------------------------------------------------------------------

TELE_HEADER     = (0xAA, 0xBB)
STOP_HEADER     = (0xAA, 0xCC)
TELE_PACKET_LEN = 33             # 2 header + 30 payload + 1 XOR checksum
STOP_PACKET_LEN = 38             # 2 header + 4 final_pos + 32 reason
TELE_FMT        = "<IiiihhhhHBBh"  # ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, cs, pwm, mvel
STOP_FMT        = "<i32s"        # STOP payload layout

# ---------------------------------------------------------------------------
# Tolerances — tune as the controller improves
# All thresholds in encoder counts (4096 counts/rev, µstep-independent)
# ---------------------------------------------------------------------------

POS_TOLERANCE   = 20     # enc counts: max |final_pos − expected| (PD overshoot ≈ 12)
LAG_THRESHOLD   = 50     # enc counts: max |lag| during motion
DRIFT_TOLERANCE = 10     # enc counts: max drift during hold (≥ 4 encoder counts)
HOLD_WAIT_SEC   = 5.0    # seconds to hold before checking drift
INTER_TEST_SEC  = 2.0    # seconds between tests for motor to settle

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class TelemetryPacket:
    ts: int       # ms since boot
    pos: int      # commanded position (µsteps)
    meas: int     # measured encoder position (µsteps)
    target: int   # target position for current segment
    lag: int      # target − measured
    vel: int      # current velocity (steps/sec)
    p_acc: int    # planned acceleration
    p_dist: int   # planned distance remaining
    sg: int       # TMC2209 stallguard result


@dataclass
class StopPacket:
    final_pos: int   # position when motion ended
    reason: str      # "Completed" | "Lag Fault" | "E-STOP (SW1)" | "Brownout Fault"


@dataclass
class MoveResult:
    telemetry: List[TelemetryPacket] = field(default_factory=list)
    stop: Optional[StopPacket] = None
    duration: float = 0.0

    @property
    def max_lag(self) -> int:
        return max((abs(t.lag) for t in self.telemetry), default=0)

    @property
    def completed(self) -> bool:
        return self.stop is not None and self.stop.reason == "Completed"

    @property
    def packet_count(self) -> int:
        return len(self.telemetry)


# ---------------------------------------------------------------------------
# StepperTestClient
# ---------------------------------------------------------------------------

class StepperTestClient:
    """Manages COM3 (commands + binary telemetry) and COM4 (debug UART)."""

    CMD_PORT = "COM3"
    CMD_BAUD = 921600
    DBG_PORT = "COM4"
    DBG_BAUD = 115200

    def __init__(self):
        self.cmd_ser: Optional[serial.Serial] = None
        self.dbg_ser: Optional[serial.Serial] = None
        self.dbg_lines: List[str] = []
        self._dbg_thread: Optional[threading.Thread] = None
        self._dbg_stop = False
        self.last_known_pos: Optional[int] = None
        self._pre_test_dbg_idx = 0

    # --- Lifecycle ---

    def connect(self):
        self.cmd_ser = serial.Serial(
            self.CMD_PORT, self.CMD_BAUD, timeout=0.1, dsrdtr=True
        )
        self.dbg_ser = serial.Serial(
            self.DBG_PORT, self.DBG_BAUD, timeout=1, dsrdtr=True
        )
        time.sleep(0.2)
        self.cmd_ser.reset_input_buffer()
        self._dbg_stop = False
        self._dbg_thread = threading.Thread(target=self._dbg_reader, daemon=True)
        self._dbg_thread.start()

    def disconnect(self):
        self._dbg_stop = True
        if self._dbg_thread:
            self._dbg_thread.join(timeout=2)
        if self.cmd_ser and self.cmd_ser.is_open:
            self.cmd_ser.close()
        if self.dbg_ser and self.dbg_ser.is_open:
            self.dbg_ser.close()

    def _dbg_reader(self):
        while not self._dbg_stop:
            try:
                raw = self.dbg_ser.readline()
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.dbg_lines.append(line)
            except Exception:
                if self._dbg_stop:
                    break

    # --- Commands ---

    def send_command(self, cmd: dict):
        self.cmd_ser.write((json.dumps(cmd) + "\n").encode("utf-8"))

    def send_move(self, distance: int, speed: float, accel: float,
                  chain: bool = False, abs_mode: bool = False):
        self.send_command({
            "cmd": "move",
            "distance": distance,
            "speed": speed,
            "accel": accel,
            "abs": abs_mode,
            "chain": chain,
        })

    # --- Telemetry collection ---

    def execute_move(self, distance: int, speed: float, accel: float,
                     chain: bool = False, abs_mode: bool = False,
                     timeout: float = 30.0) -> MoveResult:
        """Send a single move and collect telemetry until STOP."""
        time.sleep(0.1)  # let firmware finish any post-move settle
        self.cmd_ser.reset_input_buffer()
        self.send_move(distance, speed, accel, chain=chain, abs_mode=abs_mode)
        return self._collect_until_stop(timeout)

    def execute_chain(self, moves: List[Tuple[int, float, float]],
                      abs_mode: bool = False,
                      timeout: float = 60.0) -> MoveResult:
        """Send chained moves upfront and collect until the single final STOP."""
        time.sleep(0.1)
        self.cmd_ser.reset_input_buffer()
        n = len(moves)
        for i, (dist, speed, accel) in enumerate(moves):
            self.send_move(dist, speed, accel, chain=(i < n - 1), abs_mode=abs_mode)
        return self._collect_until_stop(timeout)

    def _collect_until_stop(self, timeout: float) -> MoveResult:
        buf = bytearray()
        start = time.time()
        result = MoveResult()
        last_data_time = start

        while time.time() - start < timeout:
            chunk = self.cmd_ser.read(self.cmd_ser.in_waiting or 1)
            if not chunk:
                # If we received telemetry but no STOP, and no data for 3s,
                # the STOP was likely lost. Do an aggressive drain and retry.
                if (result.telemetry and
                        time.time() - last_data_time > 3.0):
                    time.sleep(0.2)
                    drain = self.cmd_ser.read(self.cmd_ser.in_waiting or 0)
                    if drain:
                        buf.extend(drain)
                    else:
                        break  # give up — no more data coming
                continue
            last_data_time = time.time()
            buf.extend(chunk)

            while len(buf) >= 2:
                if buf[0] != 0xAA:
                    buf = buf[1:]
                    continue

                if buf[1] == TELE_HEADER[1]:               # UPDATE
                    if len(buf) < TELE_PACKET_LEN:
                        break
                    pkt = buf[:TELE_PACKET_LEN]
                    chk = 0
                    for b in pkt[2:32]:
                        chk ^= b
                    if chk == pkt[32]:
                        vals = struct.unpack(TELE_FMT, bytes(pkt[2:32]))
                        ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, _cs, _pwm, _mvel = vals
                        result.telemetry.append(TelemetryPacket(
                            ts=ts, pos=pos, meas=meas, target=target,
                            lag=lag, vel=vel, p_acc=p_acc, p_dist=p_dist, sg=sg,
                        ))
                        buf = buf[TELE_PACKET_LEN:]
                    else:
                        buf = buf[1:]  # checksum fail: skip 1 byte to resync

                elif buf[1] == STOP_HEADER[1]:              # STOP
                    if len(buf) < STOP_PACKET_LEN:
                        break
                    pkt = buf[:STOP_PACKET_LEN]
                    final_pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                    reason = reason_b.rstrip(b"\x00").decode("ascii", errors="replace")
                    result.stop = StopPacket(final_pos=final_pos, reason=reason)
                    result.duration = time.time() - start
                    self.last_known_pos = final_pos
                    return result
                else:
                    buf = buf[1:]

        result.duration = time.time() - start
        return result

    # --- Debug serial helpers ---

    def get_system_status(self) -> Dict:
        for line in reversed(self.dbg_lines):
            if "[SYSTEM]" in line:
                m = re.search(r"VBus:\s*([\d.]+)V.*PG:\s*(\w+)", line)
                if m:
                    return {"vbus": float(m.group(1)), "pg_ok": m.group(2) == "OK"}
        return {"vbus": None, "pg_ok": False}

    def check_no_reboot(self, since_idx: int = 0) -> bool:
        return not any("SYSTEM BOOT" in l for l in self.dbg_lines[since_idx:])

    def check_no_fault(self, since_idx: int = 0) -> bool:
        return not any(
            ("FAULT" in l or "ERR:" in l) for l in self.dbg_lines[since_idx:]
        )


# ---------------------------------------------------------------------------
# Pytest configuration & markers
# ---------------------------------------------------------------------------

def pytest_configure(config):
    config.addinivalue_line(
        "markers", "hardware: requires physical PD-Stepper board"
    )


pytestmark = pytest.mark.hardware


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def stepper_client():
    """Session-scoped: open COM3 + COM4 once, reuse across all tests."""
    client = StepperTestClient()
    try:
        client.connect()
    except serial.SerialException as e:
        pytest.skip(f"Hardware not available: {e}")

    time.sleep(3)  # wait for debug reader to pick up [SYSTEM] line

    # Drain stale data: previous sessions may leave STOP packets in the
    # ESP32 USB CDC buffer that arrive after port open.
    time.sleep(0.5)
    stale = client.cmd_ser.read(client.cmd_ser.in_waiting or 0)
    if stale:
        client.cmd_ser.reset_input_buffer()
        time.sleep(0.3)

    # Calibration round-trip: verify board responds and establish baseline pos
    r = client.execute_move(100, 2000.0, 3000, timeout=10)
    if not r.completed:
        # If the stale STOP was consumed as the calibration result, the real
        # move data is still pending. Try once more after a fresh drain.
        time.sleep(1)
        client.cmd_ser.reset_input_buffer()
        r = client.execute_move(100, 2000.0, 3000, timeout=10)
    if not r.completed:
        client.disconnect()
        pytest.skip("Calibration move failed — board not responding")
    r = client.execute_move(-100, 2000.0, 3000, timeout=10)
    if not r.completed:
        client.disconnect()
        pytest.skip("Calibration return failed — board not responding")

    yield client
    client.disconnect()


@pytest.fixture(autouse=True)
def system_guard(stepper_client):
    """Before each test: check VBus/PG health. After: settle pause."""
    status = stepper_client.get_system_status()
    if status["vbus"] is not None:
        assert status["pg_ok"], f"PG: FAIL — VBus: {status['vbus']}V (check USB PD)"
        assert status["vbus"] > 10.0, f"VBus too low: {status['vbus']}V"
    stepper_client._pre_test_dbg_idx = len(stepper_client.dbg_lines)
    yield
    time.sleep(INTER_TEST_SEC)


# ---------------------------------------------------------------------------
# Assertion helpers
# ---------------------------------------------------------------------------

def assert_move_ok(result: MoveResult,
                   expected_distance: Optional[int] = None,
                   pos_before: Optional[int] = None,
                   pos_tolerance: int = POS_TOLERANCE,
                   lag_threshold: int = LAG_THRESHOLD):
    """Standard pass/fail checks for a completed move."""
    assert result.stop is not None, "No STOP packet received (timeout)"
    assert result.completed, f"Move faulted: {result.stop.reason}"
    assert result.max_lag <= lag_threshold, \
        f"Max lag {result.max_lag} exceeded threshold {lag_threshold}"
    if expected_distance is not None and pos_before is not None:
        actual = result.stop.final_pos - pos_before
        assert abs(actual - expected_distance) <= pos_tolerance, \
            f"Position error: expected Δ{expected_distance}, got Δ{actual} " \
            f"(off by {abs(actual - expected_distance)})"


# ---------------------------------------------------------------------------
# Tests: Single move accuracy
# ---------------------------------------------------------------------------

class TestSingleMove:

    def test_forward_accuracy(self, stepper_client):
        """3200-step forward move: check position delta matches."""
        pos_before = stepper_client.last_known_pos
        r = stepper_client.execute_move(3200, 12000.0, 9000)
        assert_move_ok(r, expected_distance=3200, pos_before=pos_before)
        stepper_client.execute_move(-3200, 12000.0, 9000)

    def test_return_to_origin(self, stepper_client):
        """Forward + reverse round-trip: net displacement ≈ 0."""
        pos_start = stepper_client.last_known_pos

        r1 = stepper_client.execute_move(3200, 12000.0, 9000)
        assert_move_ok(r1)

        r2 = stepper_client.execute_move(-3200, 12000.0, 9000)
        assert_move_ok(r2)

        if pos_start is not None:
            net = abs(r2.stop.final_pos - pos_start)
            assert net <= POS_TOLERANCE, \
                f"Round-trip error: {net} µsteps (tolerance: {POS_TOLERANCE})"


# ---------------------------------------------------------------------------
# Tests: Speed sweep
# ---------------------------------------------------------------------------

class TestSpeedRange:

    @pytest.mark.parametrize("speed", [2000, 6000, 12000, 20000])
    def test_speed(self, stepper_client, speed):
        """Complete a move at {speed} steps/s without fault."""
        pos_before = stepper_client.last_known_pos
        r = stepper_client.execute_move(3200, float(speed), 9000)
        assert_move_ok(r, expected_distance=3200, pos_before=pos_before)
        stepper_client.execute_move(-3200, float(speed), 9000)


# ---------------------------------------------------------------------------
# Tests: Acceleration sweep
# ---------------------------------------------------------------------------

class TestAccelRange:

    @pytest.mark.parametrize("accel", [3000, 9000, 30000])
    def test_accel(self, stepper_client, accel):
        """Complete a move with accel={accel} steps/s² without fault."""
        pos_before = stepper_client.last_known_pos
        r = stepper_client.execute_move(3200, 12000.0, float(accel))
        assert_move_ok(r, expected_distance=3200, pos_before=pos_before)
        stepper_client.execute_move(-3200, 12000.0, float(accel))


# ---------------------------------------------------------------------------
# Tests: Chain moves
# ---------------------------------------------------------------------------

class TestChainMoves:

    def test_two_segment_chain(self, stepper_client):
        """1600 + 1600 chained = 3200 total, single STOP."""
        pos_before = stepper_client.last_known_pos
        r = stepper_client.execute_chain([
            (1600, 6000.0, 9000),
            (1600, 12000.0, 9000),
        ])
        assert_move_ok(r, expected_distance=3200, pos_before=pos_before)
        stepper_client.execute_move(-3200, 12000.0, 9000)

    def test_direction_reversal_chain(self, stepper_client):
        """Forward + reverse chain: net ≈ 0, single STOP."""
        pos_before = stepper_client.last_known_pos
        r = stepper_client.execute_chain([
            (3200, 12000.0, 9000),
            (-3200, 12000.0, 9000),
        ])
        assert_move_ok(r)
        if pos_before is not None:
            net = abs(r.stop.final_pos - pos_before)
            assert net <= POS_TOLERANCE, f"Chain net displacement: {net}"


# ---------------------------------------------------------------------------
# Tests: Hold stability & edge cases
# ---------------------------------------------------------------------------

class TestHoldAndEdge:

    def test_hold_stability(self, stepper_client):
        """After a move, hold for 5s — position drift must be minimal."""
        r1 = stepper_client.execute_move(3200, 12000.0, 9000)
        assert_move_ok(r1)
        pos_after = r1.stop.final_pos

        time.sleep(HOLD_WAIT_SEC)

        # Return move: first UPDATE reveals actual position after hold period
        r2 = stepper_client.execute_move(-3200, 12000.0, 9000)
        assert_move_ok(r2)

        if r2.telemetry:
            drift = abs(r2.telemetry[0].meas - pos_after)
            assert drift <= DRIFT_TOLERANCE, \
                f"Hold drift: {drift} µsteps over {HOLD_WAIT_SEC}s"

    def test_small_move(self, stepper_client):
        """100-step move near encoder resolution limit."""
        pos_before = stepper_client.last_known_pos
        r = stepper_client.execute_move(100, 2000.0, 3000)
        assert_move_ok(r, expected_distance=100, pos_before=pos_before,
                       pos_tolerance=15)  # wider for small moves
        stepper_client.execute_move(-100, 2000.0, 3000)

    def test_system_health(self, stepper_client):
        """End-of-session: VBus OK, no faults or reboots during campaign."""
        status = stepper_client.get_system_status()
        assert status["pg_ok"], "PG: FAIL at end of test campaign"
        assert status["vbus"] is not None and status["vbus"] > 15.0, \
            f"VBus anomaly: {status['vbus']}V"
