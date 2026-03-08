"""
TriggerMove.py — Send a single move command to PD-Stepper via USB serial.

Usage:
    python TriggerMove.py [--port COM3] [--distance 3200] [--speed 12000] [--accel 9000] [--abs]

Defaults: COM3, 3200 steps (1 rev @ 16 microsteps), 12000 steps/sec, 9000 steps/sec²
"""

import argparse
import json
import struct
import time
import serial

TELE_HEADER     = (0xAA, 0xBB)
STOP_HEADER     = (0xAA, 0xCC)
TELE_PACKET_LEN = 29
STOP_PACKET_LEN = 38
TELE_FMT        = "<IiiihhhhHB"
STOP_FMT        = "<i32s"


def send_cmd(ser: serial.Serial, obj: dict):
    ser.write((json.dumps(obj) + "\n").encode("utf-8"))


def wait_for_completion(ser: serial.Serial, timeout: float = 30.0):
    """Read telemetry packets until STOP packet received, printing live position."""
    buf   = bytearray()
    start = time.time()

    print(f"  {'Time':>6}  {'Target':>8}  {'Measured':>9}  {'Lag':>6}  {'Vel':>7}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*9}  {'-'*6}  {'-'*7}")

    while time.time() - start < timeout:
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
                for b in pkt[2:28]:
                    chk ^= b
                if chk == pkt[28]:
                    fields = struct.unpack(TELE_FMT, bytes(pkt[2:]))
                    ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, _ = fields
                    t_sec = ts / 1000.0
                    print(f"  {t_sec:6.2f}  {target:>8}  {meas:>9}  {lag:>6}  {vel:>7}")
                buf = buf[TELE_PACKET_LEN:]

            elif buf[1] == STOP_HEADER[1]:
                if len(buf) < STOP_PACKET_LEN:
                    break
                pkt = buf[:STOP_PACKET_LEN]
                final_pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                reason = reason_b.rstrip(b"\x00").decode("ascii", errors="replace")
                duration = time.time() - start
                print(f"\nMove complete in {duration:.2f}s — final pos: {final_pos}  reason: {reason}")
                return True
            else:
                buf = buf[1:]

    print("\nTimeout waiting for move to complete.")
    return False


def main():
    parser = argparse.ArgumentParser(description="PD-Stepper single move command")
    parser.add_argument("--port",     default="COM3",       help="Serial port (default: COM3)")
    parser.add_argument("--distance", type=int,   default=3200,  help="Steps to move (default: 3200)")
    parser.add_argument("--speed",    type=float, default=12000, help="Steps/sec (default: 12000)")
    parser.add_argument("--accel",    type=float, default=9000,  help="Steps/sec² (default: 9000)")
    parser.add_argument("--abs",      action="store_true",        help="Absolute move (default: relative)")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        print(f"ERROR: {e}")
        return

    time.sleep(1.5)  # wait for ESP32 to settle after USB connect
    ser.reset_input_buffer()

    cmd = {
        "cmd":      "move",
        "distance": args.distance,
        "speed":    args.speed,
        "accel":    args.accel,
        "abs":      args.abs,
    }
    print(f"Sending: {json.dumps(cmd)}\n")
    send_cmd(ser, cmd)

    wait_for_completion(ser)
    ser.close()


if __name__ == "__main__":
    main()
