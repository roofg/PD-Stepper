"""
DiagnoseEndVel.py — Run N back-and-forth moves and print all telemetry packets.

Shows:
  - Every telemetry packet with wall-clock and firmware timestamp
  - Highlights the last 15 packets before each STOP (the deceleration tail)
  - Gap (ms) between last UPDATE and STOP
  - Velocity at STOP entry

This helps reproduce the "last-part-missing" bug where the chart shows a
vel jump at hold entry.
"""

import argparse, json, struct, time, serial

PORT      = "COM3"
BAUD      = 921600
TELE_LEN  = 33
STOP_LEN  = 38
TELE_FMT  = "<IiiihhhhHBBh"   # ts,pos,meas,target,lag,vel,p_acc,p_dist,sg,cs,pwm,mvel
STOP_FMT  = "<i32s"
STALE_S   = 0.040
TAIL_ROWS = 20

# Encoder-count conversion constants (AS5600: 4096 counts/rev)
COUNTS_PER_REV  = 4096
DEG_PER_COUNT   = 360.0 / COUNTS_PER_REV
RPM_PER_CPS     = 60.0 / COUNTS_PER_REV

def enc_to_deg(counts): return counts * DEG_PER_COUNT
def enc_per_sec_to_rpm(cps): return cps * RPM_PER_CPS

def send_move(ser, distance, speed=12000, accel=9000, absolute=False):
    cmd = {"cmd":"move","distance":distance,"speed":speed,"accel":accel,"abs":absolute,"chain":False}
    ser.write((json.dumps(cmd)+"\n").encode())

def drain(ser):
    ser.timeout = 0.05
    while ser.read(256):
        pass
    ser.reset_input_buffer()
    ser.timeout = 0.1

def wait_move(ser, move_num, timeout=30.0):
    buf       = bytearray()
    text_line = bytearray()
    packets   = []
    t0        = time.time()

    while time.time() - t0 < timeout:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= 2:
            if buf[0] != 0xAA:
                ch = buf[0]; buf = buf[1:]
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

            if buf[1] == 0xBB:
                if len(buf) < TELE_LEN: break
                pkt = buf[:TELE_LEN]
                chk = 0
                for b in pkt[2:32]: chk ^= b
                if chk == pkt[32]:
                    f = struct.unpack(TELE_FMT, bytes(pkt[2:32]))
                    packets.append({
                        "wall": time.time()-t0,
                        "ts":f[0], "pos":f[1], "meas":f[2], "target":f[3],
                        "lag":f[4], "vel":f[5], "acc":f[6], "dist":f[7],
                        "sg":f[8], "mvel":f[11],
                    })
                buf = buf[TELE_LEN:]

            elif buf[1] == 0xCC:
                if len(buf) < STOP_LEN: break
                elapsed = time.time() - t0
                if elapsed < STALE_S:
                    print(f"  [WARN] stale STOP at {elapsed*1000:.0f}ms, discarding")
                    buf = buf[STOP_LEN:]; continue
                final_pos, reason_b = struct.unpack(STOP_FMT, bytes(buf[2:STOP_LEN]))
                reason = reason_b.rstrip(b"\x00").decode("ascii","replace")
                buf = buf[STOP_LEN:]
                return packets, final_pos, reason, elapsed
            else:
                buf = buf[1:]

    return packets, None, "TIMEOUT", time.time()-t0

def print_move_summary(move_num, packets, final_pos, reason, elapsed):
    n = len(packets)
    print(f"\n{'='*72}")
    print(f"MOVE {move_num}  pkts={n}  final_pos={enc_to_deg(final_pos):.1f}°  "
          f"reason={reason}  duration={elapsed:.3f}s")

    if not packets:
        print("  (no packets received)")
        return

    last_wall = packets[-1]["wall"]
    stop_gap  = elapsed - last_wall
    v_last    = packets[-1]["vel"]
    print(f"  last_pkt @ {last_wall*1000:.0f}ms  STOP @ {elapsed*1000:.0f}ms  "
          f"gap={stop_gap*1000:.0f}ms  last_vel={enc_per_sec_to_rpm(v_last):.1f}RPM  last_mvel={enc_per_sec_to_rpm(packets[-1]['mvel']):.1f}RPM")

    tail = packets[max(0, n-TAIL_ROWS):]
    print(f"\n  {'#':>4}  {'wall_ms':>8}  {'fw_ms':>8}  {'RPM':>7}  {'mRPM':>7}  "
          f"{'lag°':>7}  {'dist°':>7}")
    print(f"  {'-'*4}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*7}  {'-'*7}  {'-'*7}")
    t0_fw = packets[0]["ts"]
    for i, p in enumerate(tail):
        idx    = n - len(tail) + i
        fw_ms  = ((p["ts"] - t0_fw) & 0xFFFFFFFF) / 1000.0
        marker = " <<<LAST" if idx == n-1 else ""
        print(f"  {idx:>4}  {p['wall']*1000:>8.1f}  {fw_ms:>8.1f}  "
              f"{enc_per_sec_to_rpm(p['vel']):>7.1f}  {enc_per_sec_to_rpm(p['mvel']):>7.1f}  {enc_to_deg(p['lag']):>7.2f}  {enc_to_deg(p['dist']):>7.1f}{marker}")

    if n > 1:
        gaps    = [((packets[i+1]["ts"] - packets[i]["ts"]) & 0xFFFFFFFF) / 1000.0
                   for i in range(n-1)]
        max_gap = max(gaps)
        b1 = sum(1 for g in gaps if g < 12)
        b2 = sum(1 for g in gaps if 12 <= g < 20)
        b3 = sum(1 for g in gaps if 20 <= g < 50)
        b4 = sum(1 for g in gaps if g >= 50)
        print(f"\n  Gaps: <12ms={b1}  12-20ms={b2}  20-50ms={b3}  >=50ms={b4}  "
              f"max={max_gap:.1f}ms")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",  default=PORT)
    ap.add_argument("--moves", type=int,   default=6)
    ap.add_argument("--dist",  type=int,   default=3200)
    ap.add_argument("--speed", type=float, default=12000)
    ap.add_argument("--accel", type=float, default=9000)
    args = ap.parse_args()

    print(f"Connecting {args.port} @ {BAUD}...")
    ser = serial.Serial(args.port, BAUD, timeout=0.1, dsrdtr=True)
    drain(ser)
    print(f"Ready. {args.moves} moves of ±{args.dist} steps "
          f"@ speed={args.speed} accel={args.accel}\n")

    for i in range(args.moves):
        target = args.dist * (1 if i % 2 == 0 else -1)
        print(f"\n--- Move {i+1}/{args.moves}: {target:+d} steps ---")
        send_move(ser, target, args.speed, args.accel)
        pkts, final_pos, reason, elapsed = wait_move(ser, i+1)
        print_move_summary(i+1, pkts, final_pos, reason, elapsed)
        time.sleep(0.5)

    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
