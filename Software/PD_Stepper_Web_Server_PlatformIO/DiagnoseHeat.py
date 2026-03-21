"""
DiagnoseHeat.py — Motor thermal diagnostic for PD-Stepper

Sends a move command then monitors the AUX debug UART (Serial1, COM4) for 60 s
during the hold phase, parsing [TMC] and [HOLD] lines to build a thermal report.

Usage:
    python DiagnoseHeat.py [--port COM3] [--debug COM4] [--duration 60]

Expected output on COM4:
    [SYSTEM] VBus: 20.01V, PG: OK, Core: 0
    [TMC] CS:18/31 Standstill:0 StealthChop:1 OT:0 PWM:142
    [HOLD] Active:1 State:CORRECTING StepDelta:12/s
    ...after 500 ms in deadband...
    [HOLD] Active:1 State:SETTLED StepDelta:0/s
    [TMC] CS:8/31 Standstill:1 StealthChop:1 OT:0 PWM:48
"""

import argparse
import json
import re
import serial
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

MOVE_CMD = json.dumps({
    "cmd": "move",
    "distance": 6400,   # 2 full rotations at 32 µsteps
    "speed": 8000.0,
    "accel": 3000.0,
    "abs": False,
}).encode() + b"\n"


@dataclass
class TmcSample:
    ts: float
    cs: int          # current_scaling 0–31
    standstill: int  # 1 when TMC detected standstill
    stealth: int
    ot_warn: int
    ot_shutdown: int
    pwm: int


@dataclass
class HoldSample:
    ts: float
    active: int
    state: str       # CORRECTING or SETTLED
    step_delta: int


@dataclass
class DiagSession:
    tmc_samples: list = field(default_factory=list)
    hold_samples: list = field(default_factory=list)
    stop_reason: Optional[str] = None


# Regex patterns for Serial1 debug lines
RE_TMC = re.compile(
    r"\[TMC\] CS:(\d+)/31 Standstill:(\d) StealthChop:(\d) OT:(\d)"
    r"(?P<shutdown> SHUTDOWN)? PWM:(\d+)"
)
RE_HOLD = re.compile(
    r"\[HOLD\] Active:(\d) State:(\w+) StepDelta:(\d+)/s"
)


def parse_line(line: str, ts: float, session: DiagSession) -> None:
    m = RE_TMC.search(line)
    if m:
        session.tmc_samples.append(TmcSample(
            ts=ts,
            cs=int(m.group(1)),
            standstill=int(m.group(2)),
            stealth=int(m.group(3)),
            ot_warn=int(m.group(4)),
            ot_shutdown=1 if m.group("shutdown") else 0,
            pwm=int(m.group(6)),
        ))
        return

    m = RE_HOLD.search(line)
    if m:
        session.hold_samples.append(HoldSample(
            ts=ts,
            active=int(m.group(1)),
            state=m.group(2),
            step_delta=int(m.group(3)),
        ))


def monitor_debug(debug_port: str, session: DiagSession, stop_event: threading.Event) -> None:
    """Read and parse Serial1 output until stop_event is set."""
    try:
        with serial.Serial(debug_port, 115200, timeout=1.0) as dbg:
            start = time.monotonic()
            while not stop_event.is_set():
                raw = dbg.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                ts = time.monotonic() - start
                if line:
                    print(f"  [{ts:6.1f}s] {line}")
                parse_line(line, ts, session)
    except serial.SerialException as e:
        print(f"DEBUG port error: {e}", file=sys.stderr)


def send_move(cmd_port: str) -> None:
    """Send a move command over the USB CDC command port."""
    try:
        with serial.Serial(cmd_port, 921600, timeout=2.0, dsrdtr=True) as cmd:
            time.sleep(0.2)  # brief settle after open
            cmd.write(MOVE_CMD)
            cmd.flush()
            print(f"Move command sent on {cmd_port}.")
    except serial.SerialException as e:
        print(f"CMD port error: {e}", file=sys.stderr)
        sys.exit(1)


def print_report(session: DiagSession) -> None:
    print("\n" + "=" * 60)
    print("THERMAL DIAGNOSTIC REPORT")
    print("=" * 60)

    tmc = session.tmc_samples
    hold = session.hold_samples

    if not tmc:
        print("No [TMC] samples received — check COM4 connection.")
        return

    # Settle transition timing
    settled_samples = [h for h in hold if h.state == "SETTLED"]
    correcting_samples = [h for h in hold if h.state == "CORRECTING" and h.active == 1]

    if settled_samples:
        first_settled = settled_samples[0].ts
        print(f"\n✅ SETTLED state reached at {first_settled:.1f}s after start")
    else:
        print("\n❌ SETTLED state never reached during monitoring window")
        print("   → Hold state machine may not be working; check encoder noise or deadband width")

    # Standstill detection
    standstill_samples = [t for t in tmc if t.standstill == 1]
    if standstill_samples:
        first_ss = standstill_samples[0].ts
        print(f"✅ TMC2209 standstill detected at {first_ss:.1f}s")
    else:
        print("❌ TMC2209 never detected standstill — step pulses still flowing during hold")
        print("   → Motor is running at IRUN current the whole time (high heat)")

    # Current scaling before/after settle
    if len(tmc) >= 2:
        hold_tmc = [t for t in tmc if t.ts > 2.0]  # skip early motion samples
        if hold_tmc:
            cs_values = [t.cs for t in hold_tmc]
            cs_min = min(cs_values)
            cs_max = max(cs_values)
            pwm_values = [t.pwm for t in hold_tmc]
            print(f"\nCurrent scaling (CS) during hold: min={cs_min}/31, max={cs_max}/31")
            print(f"PWM scale sum during hold:         min={min(pwm_values)}, max={max(pwm_values)}")

            if settled_samples:
                settle_ts = settled_samples[0].ts
                before = [t.cs for t in hold_tmc if t.ts < settle_ts]
                after_settle = [t.cs for t in hold_tmc if t.ts >= settle_ts]
                if before and after_settle:
                    print(f"\nCS before SETTLED: avg={sum(before)/len(before):.1f}")
                    print(f"CS after  SETTLED: avg={sum(after_settle)/len(after_settle):.1f}")
                    reduction = (1 - (sum(after_settle)/len(after_settle)) /
                                     (sum(before)/len(before))) * 100
                    print(f"→ Current scaling reduced by ~{reduction:.0f}% after settling")

    # Over-temp warnings
    ot = [t for t in tmc if t.ot_warn or t.ot_shutdown]
    if ot:
        print(f"\n⚠️  Over-temperature events: {len(ot)}")
        for s in ot[:3]:
            print(f"   t={s.ts:.1f}s  warn={s.ot_warn}  shutdown={s.ot_shutdown}")
    else:
        print("\n✅ No over-temperature events")

    # Step delta during settled hold
    settled_steps = [h.step_delta for h in hold if h.state == "SETTLED" and h.active == 1]
    if settled_steps:
        avg_delta = sum(settled_steps) / len(settled_steps)
        print(f"\nStep delta during SETTLED hold: avg={avg_delta:.0f}/s  max={max(settled_steps)}/s")
        if avg_delta < 5:
            print("✅ Step pulses essentially silent during settled hold")
        else:
            print("⚠️  Step pulses still present during settled hold — encoder noise exceeding deadband?")

    print("\n" + "=" * 60)


def main() -> None:
    parser = argparse.ArgumentParser(description="PD-Stepper thermal diagnostic")
    parser.add_argument("--port",     default="COM3", help="Command port (USB CDC, 921600)")
    parser.add_argument("--debug",    default="COM4", help="Debug UART port (Serial1, 115200)")
    parser.add_argument("--duration", type=int, default=60, help="Monitor duration in seconds")
    parser.add_argument("--no-move",  action="store_true", help="Skip sending move command (monitor only)")
    args = parser.parse_args()

    session = DiagSession()
    stop_event = threading.Event()

    print(f"Starting debug monitor on {args.debug} (115200 baud)...")
    monitor_thread = threading.Thread(
        target=monitor_debug,
        args=(args.debug, session, stop_event),
        daemon=True,
    )
    monitor_thread.start()
    time.sleep(0.5)  # let monitor open before triggering move

    if not args.no_move:
        print(f"Sending move command on {args.port}...")
        send_move(args.port)

    print(f"Monitoring for {args.duration}s — watch for CORRECTING→SETTLED transition...\n")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    stop_event.set()
    monitor_thread.join(timeout=2.0)

    print_report(session)


if __name__ == "__main__":
    main()
