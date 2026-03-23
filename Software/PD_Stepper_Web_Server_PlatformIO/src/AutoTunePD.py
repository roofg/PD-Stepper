"""
AutoTunePD.py — Automated PD gain sweep for PD-Stepper.

Updated for the new PD + feedforward + phase-lead architecture:
  - Uses "set_pd"          to set Kp and Kd
  - Uses "set_phase_lead"  to set Kv
  - Scores each run by: overshoot (weighted ×10), average position error,
    and move duration (weighted ×100). Lag/E-STOP faults add a heavy penalty.

Usage:
    python AutoTunePD.py --port COM5

The script performs a grid search over Kp, Kd, and optionally Kv, reporting
the best-scoring combination at the end.  All gains default to conservative
starting values — increase ranges only after confirming the motor is stable.
"""

import argparse
import json
import time
import serial
import struct

TELE_HEADER     = (0xAA, 0xBB)
STOP_HEADER     = (0xAA, 0xCC)
TELE_PACKET_LEN = 33
STOP_PACKET_LEN = 38
TELE_FMT        = "<IiiihhhhHBBh"  # ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, cs, pwm, mvel
STOP_FMT        = "<i32s"         # final_pos, reason


def send_cmd(ser: serial.Serial, obj: dict):
    ser.write((json.dumps(obj) + "\n").encode("utf-8"))
    time.sleep(0.05)


def run_test_move(ser: serial.Serial, kp: float, kd: float, kv: float,
                  accel: float, speed: float, distance: int = 12800):
    """Send one test move and collect telemetry until STOP packet.

    Returns a dict with move metrics, or None on timeout.
    """
    # Apply gains
    send_cmd(ser, {"cmd": "set_pd",          "kp": kp, "kd": kd})
    send_cmd(ser, {"cmd": "set_phase_lead",  "kv": kv})

    # Command move
    send_cmd(ser, {"cmd": "move", "distance": distance,
                   "accel": accel, "speed": speed})

    start_time   = time.time()
    buf          = bytearray()
    max_overshoot = 0
    total_error  = 0
    samples      = 0

    while True:
        if time.time() - start_time > 10.0:
            return None  # timeout

        try:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= 2:
                if buf[0] != 0xAA:
                    buf = buf[1:]
                    continue

                if buf[1] == TELE_HEADER[1]:
                    if len(buf) < TELE_PACKET_LEN:
                        break
                    pkt = buf[:TELE_PACKET_LEN]
                    chk = 0
                    for b in pkt[2:32]:
                        chk ^= b
                    if chk == pkt[32]:
                        fields = struct.unpack(TELE_FMT, bytes(pkt[2:32]))
                        ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, _cs, _pwm, _mvel = fields
                        error = abs(lag)
                        total_error += error
                        samples += 1
                        # Overshoot: encoder past target after planner has finished
                        if p_dist == 0 and lag < 0:
                            os = abs(lag)
                            if os > max_overshoot:
                                max_overshoot = os
                    buf = buf[TELE_PACKET_LEN:]

                elif buf[1] == STOP_HEADER[1]:
                    if len(buf) < STOP_PACKET_LEN:
                        break
                    pkt = buf[:STOP_PACKET_LEN]
                    final_pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                    reason = reason_b.rstrip(b"\x00").decode("ascii", errors="replace")
                    duration  = time.time() - start_time
                    avg_error = total_error / samples if samples > 0 else 9999
                    return {
                        "duration":   duration,
                        "overshoot":  max_overshoot,
                        "avg_error":  avg_error,
                        "reason":     reason,
                    }
                else:
                    buf = buf[1:]

        except Exception as e:
            print(f"\nError during move: {e}")
            return None


def wait_for_stop(ser: serial.Serial, timeout: float = 6.0):
    """Drain serial until a STOP packet arrives (for the return move)."""
    start = time.time()
    buf   = bytearray()
    while time.time() - start < timeout:
        try:
            if ser.in_waiting:
                buf.extend(ser.read(ser.in_waiting))
                if b"\xaa\xcc" in buf:
                    return
            else:
                time.sleep(0.01)
        except Exception:
            return


def main():
    parser = argparse.ArgumentParser(description="PD-Stepper AutoTune PD gains")
    parser.add_argument("--port",     required=True,          help="Serial port (e.g. COM5)")
    parser.add_argument("--accel",    type=float, default=9000, help="Acceleration for test moves")
    parser.add_argument("--speed",    type=float, default=10000, help="Speed for test moves")
    parser.add_argument("--distance", type=int,   default=12800, help="Relative move distance (steps)")
    args = parser.parse_args()

    print(f"[USB] Opening {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        print(f"[USB] ERROR: {e}")
        return

    time.sleep(1.0)  # let ESP32 settle after connect
    print(f"Connected. Test move: {args.distance} steps @ speed={args.speed}, accel={args.accel}\n")

    # --- Gain search ranges ---
    # Start conservative. Widen after confirming stability.
    kp_range  = [1.0, 2.0, 3.0, 5.0]
    kd_range  = [0.0, 0.05, 0.1, 0.2]
    kv_range  = [0.0]        # add 0.002, 0.005 etc. after basic PD is tuned
    accel     = args.accel
    speed     = args.speed
    distance  = args.distance

    best_score  = float("inf")
    best_params = None
    results     = []

    for kv in kv_range:
        for kp in kp_range:
            for kd in kd_range:
                label = f"Kp={kp:.2f}  Kd={kd:.3f}  Kv={kv:.4f}"
                print(f"  Testing {label} ...", end="", flush=True)

                res = run_test_move(ser, kp, kd, kv, accel, speed, distance)
                if res is None:
                    print(" TIMEOUT")
                    continue

                # Return move
                send_cmd(ser, {"cmd": "move", "distance": -distance,
                               "accel": accel, "speed": speed})
                wait_for_stop(ser)

                # Score: lower is better
                score = (res["overshoot"] * 10.0
                         + res["avg_error"]
                         + res["duration"] * 100.0)
                if "Fault" in res["reason"] or "STOP" in res["reason"]:
                    score += 10000.0

                print(f"  score={score:.1f}  "
                      f"OS={res['overshoot']}  "
                      f"AvgErr={res['avg_error']:.1f}  "
                      f"t={res['duration']:.2f}s  "
                      f"[{res['reason']}]")

                results.append((score, kp, kd, kv, res))
                if score < best_score:
                    best_score  = score
                    best_params = (kp, kd, kv)

                time.sleep(0.5)

    print("\n=== AutoTune Results ===")
    if best_params:
        kp, kd, kv = best_params
        print(f"  Best Kp    : {kp}")
        print(f"  Best Kd    : {kd}")
        print(f"  Best Kv    : {kv}")
        print(f"  Best Score : {best_score:.2f}")
        print(f"\n  Apply with:")
        print(f'    {{"cmd":"set_pd","kp":{kp},"kd":{kd}}}')
        if kv != 0.0:
            print(f'    {{"cmd":"set_phase_lead","kv":{kv}}}')
    else:
        print("  No successful tests. Check wiring and serial port.")

    ser.close()


if __name__ == "__main__":
    main()
