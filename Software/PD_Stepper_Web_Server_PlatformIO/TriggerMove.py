"""
TriggerMove.py — Send a single move command to PD-Stepper via USB serial.

Usage:
    python TriggerMove.py [--port COM3] [--distance 3200] [--speed 12000] [--accel 9000] [--abs] [--chain]

Defaults: COM3, 3200 steps (1 rev @ 16 microsteps), 12000 steps/sec, 9000 steps/sec²

--chain: mark this move as chained (do not stop motor at end; next queued command continues).
         Usually combined with subsequent commands sent before this one finishes.
"""

import argparse
import json
import struct
import time
import serial

TELE_HEADER     = (0xAA, 0xBB)
STOP_HEADER     = (0xAA, 0xCC)
TELE_PACKET_LEN = 33
STOP_PACKET_LEN = 38
TELE_FMT        = "<IiiihhhhHBBh"  # ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, cs, pwm, mvel
STOP_FMT        = "<i32s"

# The firmware enforces a 50 ms settle delay before it can send any STOP packet.
# A STOP arriving sooner than this is a stale packet left in the OS serial buffer
# from a previous run.  40 ms is safely below the firmware minimum (~50 ms) while
# remaining well above typical stale-packet arrival (~10–25 ms after port open).
STALE_GUARD_S   = 0.040


def send_cmd(ser: serial.Serial, obj: dict):
    ser.write((json.dumps(obj) + "\n").encode("utf-8"))


def wait_for_completion(ser: serial.Serial, timeout: float = 30.0):
    """Read telemetry packets until STOP packet received, printing live position."""
    buf   = bytearray()
    start = time.time()

    print(f"  {'Time':>6}  {'Target':>8}  {'Measured':>9}  {'Lag':>6}  {'Vel':>7}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*9}  {'-'*6}  {'-'*7}")

    # Accumulate a text line buffer for firmware debug prints (DBG:... lines)
    text_line = bytearray()

    while time.time() - start < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= 2:
            # Collect ASCII text until newline for firmware debug lines
            if buf[0] != 0xAA:
                ch = buf[0]
                buf = buf[1:]
                if ch == ord('\n'):
                    line = text_line.decode("ascii", errors="replace").strip()
                    if line:
                        print(f"  [FW] {line}")
                    text_line = bytearray()
                elif ch >= 0x20:  # printable ASCII
                    text_line.append(ch)
                else:
                    text_line = bytearray()
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
                    t_sec = ts / 1000.0
                    print(f"  {t_sec:6.2f}  {target:>8}  {meas:>9}  {lag:>6}  {vel:>7}")
                buf = buf[TELE_PACKET_LEN:]

            elif buf[1] == STOP_HEADER[1]:
                if len(buf) < STOP_PACKET_LEN:
                    break
                elapsed = time.time() - start
                if elapsed < STALE_GUARD_S:
                    # Arrived before the firmware's minimum response time — stale
                    # OS-buffered packet from a previous run.  Discard and continue.
                    print(f"  [WARN] Discarding stale STOP packet (arrived at {elapsed*1000:.0f}ms)")
                    buf = buf[STOP_PACKET_LEN:]
                    continue
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
    parser.add_argument("--chain",    action="store_true",        help="Mark move as chained (do not stop at end)")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at 921600 baud...")
    try:
        # dsrdtr=True keeps DTR/DSR lines stable — prevents the ESP32 from
        # resetting when pyserial toggles DTR on port open/close.
        ser = serial.Serial(args.port, 921600, timeout=0.1, dsrdtr=True)
    except Exception as e:
        print(f"ERROR: {e}")
        return

    # Dynamic drain: the ESP32 retransmits bytes buffered in its CDC TX FIFO
    # when the host reconnects.  Retransmission can arrive tens to hundreds of
    # milliseconds after port open — a fixed sleep is unreliable on Windows.
    # Read with a short (50 ms) timeout and loop until 50 ms of silence; that
    # guarantees all stale bytes are consumed before we send the move command.
    ser.timeout = 0.05
    while ser.read(256):
        pass
    ser.reset_input_buffer()
    ser.timeout = 0.1  # restore normal read timeout

    cmd = {
        "cmd":      "move",
        "distance": args.distance,
        "speed":    args.speed,
        "accel":    args.accel,
        "abs":      args.abs,
        "chain":    args.chain,
    }
    print(f"Sending: {json.dumps(cmd)}\n")
    send_cmd(ser, cmd)

    wait_for_completion(ser)
    ser.close()


if __name__ == "__main__":
    main()
