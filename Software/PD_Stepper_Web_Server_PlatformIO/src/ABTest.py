import argparse
import json
import time
import serial
import struct

TELE_HEADER      = (0xAA, 0xBB)
STOP_HEADER      = (0xAA, 0xCC)
TELE_PACKET_LEN  = 27
STOP_PACKET_LEN  = 38
STOP_FMT         = "<i32s"

def run_move(ser: serial.Serial, telemetry_on):
    mode = "ON" if telemetry_on else "OFF"
    print(f"\n--- Starting Move with Telemetry {mode} ---")
    
    try:
        # 1. Toggle Telemetry
        cmd_tele = {"cmd": "telemetry", "enabled": telemetry_on}
        ser.write((json.dumps(cmd_tele) + "\n").encode('utf-8'))
        
        # 2. Command Move
        cmd = {"cmd": "move", "distance": 25600, "accel": 9000, "speed": 10000}
        ser.write((json.dumps(cmd) + "\n").encode('utf-8'))
        
        start_time = time.time()
        buf = bytearray()
        
        while True:
            try:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk: continue
                buf.extend(chunk)

                while len(buf) >= 2:
                    if buf[0] != 0xAA:
                        buf = buf[1:]
                        continue
                    
                    if buf[1] == TELE_HEADER[1]:
                        if len(buf) < TELE_PACKET_LEN: break
                        buf = buf[TELE_PACKET_LEN:]
                        if telemetry_on:
                            pass
                    elif buf[1] == STOP_HEADER[1]:
                        if len(buf) < STOP_PACKET_LEN: break
                        pkt = buf[:STOP_PACKET_LEN]
                        pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                        reason = reason_b.rstrip(b'\x00').decode("ascii", errors="replace")
                        duration = time.time() - start_time
                        print(f"\nMove finished in {duration:.2f}s. Reason: {reason}")
                        return
                    else:
                        buf = buf[1:]
            except Exception as e:
                print(f"Error during move: {e}")
                break
    except Exception as e:
        print(f"Failed to communicate: {e}")

def main():
    parser = argparse.ArgumentParser(description="PD-Stepper A/B Test")
    parser.add_argument("--port", required=True, help="Serial port")
    args = parser.parse_args()

    print(f"[USB] Opening {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        print(f"[USB] ERROR: {e}")
        return

    print("PD-Stepper A/B Test: Motor Smoothness Comparison")
    
    # Test A: Telemetry ON
    run_move(ser, True)
    
    print("\nWaiting 2 seconds for motor to cool/settle...")
    time.sleep(2)
    
    # Test B: Telemetry OFF
    run_move(ser, False)

if __name__ == "__main__":
    main()
