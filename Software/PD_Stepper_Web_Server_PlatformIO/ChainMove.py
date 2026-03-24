"""
ChainMove.py — Send a sequence of chained moves to PD-Stepper via USB serial.

All moves are sent upfront before motion starts. Each move except the last is
flagged chain=true so the firmware transitions without stopping the motor or
disabling the TMC driver. One STOP packet is received at the end of the chain.

Usage:
    python ChainMove.py --move dist:speed:accel --move dist:speed:accel [...]

    Each --move argument is colon-separated: distance:speed:accel
      distance  Steps (positive = forward, negative = reverse)
      speed     Steps/sec
      accel     Steps/sec²

Examples:
    # Two same-direction moves at different speeds (smooth velocity transition)
    python ChainMove.py --move 1600:6000:9000 --move 1600:12000:9000

    # Forward then reverse (direction-reversal chain: stops at boundary, immediate restart)
    python ChainMove.py --move 3200:12000:9000 --move -3200:12000:9000

    # Three-move profile: ramp in, cruise, ramp out
    python ChainMove.py --move 800:12000:3000 --move 2400:12000:50000 --move 800:6000:3000

Options:
    --port PORT    Serial port (default: COM3)
    --abs          Treat distances as absolute positions
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

# Encoder-count conversion constants (AS5600: 4096 counts/rev)
COUNTS_PER_REV  = 4096
DEG_PER_COUNT   = 360.0 / COUNTS_PER_REV
RPM_PER_CPS     = 60.0 / COUNTS_PER_REV

def enc_to_deg(counts): return counts * DEG_PER_COUNT
def enc_per_sec_to_rpm(cps): return cps * RPM_PER_CPS


def send_cmd(ser: serial.Serial, obj: dict) -> None:
    ser.write((json.dumps(obj) + "\n").encode("utf-8"))


def wait_for_chain_completion(ser: serial.Serial, n_moves: int, timeout: float = 120.0):
    """Read telemetry until the single STOP packet that ends the whole chain."""
    buf        = bytearray()
    start      = time.time()
    text_line  = bytearray()

    print(f"  {'Time':>8}  {'Target°':>8}  {'Meas°':>9}  {'Lag°':>7}  {'RPM':>7}")
    print(f"  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*7}  {'-'*7}")

    while time.time() - start < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= 2:
            if buf[0] != 0xAA:
                ch = buf[0]
                buf = buf[1:]
                if ch == ord('\n'):
                    line = text_line.decode("ascii", errors="replace").strip()
                    if line:
                        print(f"  [FW] {line}")
                    text_line = bytearray()
                elif ch >= 0x20:
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
                    print(f"  {t_sec:8.2f}  {enc_to_deg(target):>8.1f}  {enc_to_deg(meas):>9.1f}  {enc_to_deg(lag):>7.2f}  {enc_per_sec_to_rpm(vel):>7.1f}")
                buf = buf[TELE_PACKET_LEN:]

            elif buf[1] == STOP_HEADER[1]:
                if len(buf) < STOP_PACKET_LEN:
                    break
                pkt = buf[:STOP_PACKET_LEN]
                final_pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                reason   = reason_b.rstrip(b"\x00").decode("ascii", errors="replace")
                duration = time.time() - start
                print(f"\nChain complete in {duration:.2f}s — final pos: {enc_to_deg(final_pos):.1f}°  reason: {reason}")
                return True
            else:
                buf = buf[1:]

    print("\nTimeout waiting for chain to complete.")
    return False


def parse_move(s: str):
    parts = s.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"--move must be dist:speed:accel, got: '{s}'"
        )
    try:
        return int(parts[0]), float(parts[1]), float(parts[2])
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"Invalid --move value '{s}': {e}")


def _fix_negative_move_args(argv):
    """Merge '--move -N:s:a' into '--move=-N:s:a' so argparse doesn't treat
    the negative distance as an unknown flag."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] == "--move" and i + 1 < len(argv) \
                and argv[i + 1].startswith("-") and ":" in argv[i + 1]:
            out.append(f"--move={argv[i + 1]}")
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def main():
    import sys
    sys.argv = sys.argv[:1] + _fix_negative_move_args(sys.argv[1:])
    parser = argparse.ArgumentParser(
        description="PD-Stepper chained move sequence",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port",  default="COM3", help="Serial port (default: COM3)")
    parser.add_argument("--abs",   action="store_true", help="Treat distances as absolute positions")
    parser.add_argument("--move",  dest="moves", metavar="dist:speed:accel",
                        action="append", required=True, type=parse_move,
                        help="Move spec (repeatable). dist:speed:accel")
    args = parser.parse_args()

    if len(args.moves) < 2:
        parser.error("Specify at least 2 --move arguments to chain")

    print(f"Connecting to {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1, dsrdtr=True)
    except Exception as e:
        print(f"ERROR: {e}")
        return

    time.sleep(0.1)
    ser.reset_input_buffer()

    n = len(args.moves)
    print(f"Sending {n}-move chain upfront:")
    for i, (dist, speed, accel) in enumerate(args.moves):
        is_last = (i == n - 1)
        cmd = {
            "cmd":      "move",
            "distance": dist,
            "speed":    speed,
            "accel":    accel,
            "abs":      args.abs,
            "chain":    not is_last,   # all but last are chained
        }
        print(f"  [{i+1}/{n}] {json.dumps(cmd)}")
        send_cmd(ser, cmd)

    print()
    wait_for_chain_completion(ser, n_moves=n)
    ser.close()


if __name__ == "__main__":
    main()
