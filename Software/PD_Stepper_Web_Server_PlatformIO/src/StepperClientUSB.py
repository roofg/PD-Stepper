"""
StepperClientUSB.py
====================
Sends move commands over WebSocket (Wi-Fi) and receives high-speed binary
telemetry over the USB serial link.

Wire protocol (little-endian binary, 27-byte packets):
  [0]   0xAA  - header byte 0
  [1]   0xBB  - header byte 1 (telemetry)
  ...
  [26]  uint8   checksum (XOR of bytes 2..25)

STOP packet (38 bytes):
  [0]   0xAA  - header byte 0
  [1]   0xCC  - header byte 1 (stop)
  ...
"""

import argparse
import struct
import serial
import json
import threading
import time

TELE_HEADER      = (0xAA, 0xBB)
STOP_HEADER      = (0xAA, 0xCC)
TELE_PACKET_LEN  = 29
STOP_PACKET_LEN  = 38
TELE_FMT         = "<IiiihhhhHB"  # uint32, int32×3, int16×4, uint16, uint8
STOP_FMT         = "<i32s"       # int32, char[32]

stop_event = threading.Event()

def serial_reader(ser: serial.Serial):
    print("[USB] Connected. Waiting for telemetry...\n")
    buf = bytearray()

    while not stop_event.is_set():
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk: continue
        buf.extend(chunk)

        while len(buf) >= 2:
            # Skip until we find the magic header byte
            if buf[0] != 0xAA:
                buf = buf[1:]
                continue
            
            # Check the second byte for packet type
            if buf[1] == TELE_HEADER[1]:
                if len(buf) < TELE_PACKET_LEN: break
                pkt = buf[:TELE_PACKET_LEN]
                
                # XOR checksum validation
                chk = 0
                for b in pkt[2:28]: chk ^= b
                if chk == pkt[28]:
                    fields = struct.unpack(TELE_FMT, bytes(pkt[2:]))
                    ts, pos, meas, target, lag, vel, p_acc, p_dist, sg_result, _ = fields
                    # (Optional: print if needed, but keeping it light for performance)
                    print(f"[{ts/1e6:8.3f}s] P:{pos:7} M:{meas:7} T:{target:7} L:{lag:4} V:{vel:5} A:{p_acc:5} Rem:{p_dist:6} SG:{sg_result:4}")
                
                buf = buf[TELE_PACKET_LEN:]
                
            elif buf[1] == STOP_HEADER[1]:
                if len(buf) < STOP_PACKET_LEN: break
                pkt = buf[:STOP_PACKET_LEN]
                pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                reason = reason_b.rstrip(b'\x00').decode("ascii", errors="replace")
                print(f"\n[USB] *** STOPPED *** reason='{reason}' final_pos={pos}")
                buf = buf[STOP_PACKET_LEN:]
                stop_event.set()
                return
            else:
                # 0xAA followed by junk — skip the 0xAA and resync
                buf = buf[1:]

    ser.close()
    print("[USB] Serial port closed.")

def send_move(ser: serial.Serial, distance, accel, speed):
    cmd = {"cmd": "move", "distance": distance, "accel": accel, "speed": speed}
    cmd_str = json.dumps(cmd) + "\n"
    ser.write(cmd_str.encode('utf-8'))
    print(f"[USB] Sent: {cmd}")

    while not stop_event.is_set():
        time.sleep(0.1)

def main():
    parser = argparse.ArgumentParser(description="PD-Stepper USB Telemetry Client")
    parser.add_argument("--port", required=True, help="Serial port")
    parser.add_argument("--distance", type=int, default=25600)
    parser.add_argument("--accel", type=float, default=9000)
    parser.add_argument("--speed", type=float, default=10000)
    args = parser.parse_args()

    print(f"[USB] Opening {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        print(f"[USB] ERROR: {e}")
        return

    t = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    t.start()

    # Small delay to ensure the reader thread is ready and ESP32 isn't rebooting after serial port open
    time.sleep(1)

    try:
        send_move(ser, args.distance, args.accel, args.speed)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
        stop_event.set()

    print("[INFO] Done.")

if __name__ == "__main__":
    main()
