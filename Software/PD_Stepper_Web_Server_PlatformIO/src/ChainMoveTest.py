"""
ChainMoveTest.py
================
Diagnostic script to measure WebSocket closure latency and detect reboots.
Chains two long moves and logs high-resolution timestamps.
Special feature: Listens for ESP32 boot messages to detect reboots during motion.
"""

import argparse
import struct
import serial
import json
import threading
import time
from datetime import datetime

TELE_HEADER      = (0xAA, 0xBB)
STOP_HEADER      = (0xAA, 0xCC)
TELE_PACKET_LEN  = 33
STOP_PACKET_LEN  = 38
TELE_FMT         = "<IiiihhhhHBBh"  # ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, cs, pwm, mvel
STOP_FMT         = "<i32s"

stop_event = threading.Event()
reboot_detected = threading.Event()

def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}")

def serial_reader(ser: serial.Serial):
    log("[USB] Connected. Waiting for telemetry/boot...")
    buf = bytearray()

    while not stop_event.is_set() or not threading.main_thread().is_alive():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except:
            break
            
        if not chunk: continue
        
        # Check for plain text (reboot messages)
        try:
            text = chunk.decode("ascii", errors="ignore")
            if "[SERIAL] Ready" in text:
                log("\n!!! DETECTED ESP32 REBOOT !!!")
                reboot_detected.set()
                stop_event.set()
        except:
            pass
            
        buf.extend(chunk)

        while len(buf) >= 2:
            if buf[0] != 0xAA:
                buf = buf[1:]
                continue
            
            if buf[1] == TELE_HEADER[1]:
                if len(buf) < TELE_PACKET_LEN: break
                buf = buf[TELE_PACKET_LEN:]
                
            elif buf[1] == STOP_HEADER[1]:
                if len(buf) < STOP_PACKET_LEN: break
                pkt = buf[:STOP_PACKET_LEN]
                pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                reason = reason_b.rstrip(b'\x00').decode("ascii", errors="replace")
                log(f"[USB] *** STOPPED *** reason='{reason}' final_pos={pos}")
                buf = buf[STOP_PACKET_LEN:]
                stop_event.set()
                break
            else:
                buf = buf[1:]

    ser.close()
    log("[USB] Serial port closed.")

def perform_move(ser: serial.Serial, distance, accel, speed):
    global stop_event
    stop_event.clear()
    
    cmd = {"cmd": "move", "distance": distance, "accel": accel, "speed": speed}
    log(f"[USB] Sending Move Command: {cmd}")
    cmd_str = json.dumps(cmd) + "\n"
    ser.write(cmd_str.encode('utf-8'))
    
    t0 = time.time()
    while not stop_event.is_set():
        if reboot_detected.is_set():
            return False
        time.sleep(0.01)
    
    return True

def run_test(ser, args):
    try:
        log("--- STARTING MOVE 1 ---")
        if not perform_move(ser, args.distance, args.accel, args.speed):
            return
        
        log("Waiting 1s for stabilization...")
        time.sleep(1.0)

        log("--- STARTING MOVE 2 ---")
        if not perform_move(ser, args.distance, args.accel, args.speed):
            return

    except Exception as e:
        log(f"[USB] Error: {e}")
    finally:
        stop_event.set()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--distance", type=int, default=51200)
    parser.add_argument("--accel", type=float, default=9000)
    parser.add_argument("--speed", type=float, default=12000)
    args = parser.parse_args()

    log(f"[USB] Opening {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        log(f"[USB] ERROR: {e}")
        return

    t = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    t.start()

    time.sleep(1)

    try:
        run_test(ser, args)
    except KeyboardInterrupt:
        log("Interrupted.")
    
    log("Test Finished.")

if __name__ == "__main__":
    main()
