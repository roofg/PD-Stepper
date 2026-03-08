import argparse
import json
import time
import serial
import struct
import math

TELE_HEADER      = (0xAA, 0xBB)
STOP_HEADER      = (0xAA, 0xCC)
TELE_PACKET_LEN  = 29
STOP_PACKET_LEN  = 38
TELE_FMT         = "<IiiihhhhHB"
STOP_FMT         = "<i32s"

def run_test_move(ser: serial.Serial, kp: float, ki: float, accel: float):
    # 1. Set PID
    cmd_pid = {"cmd": "set_pid", "kp": kp, "ki": ki}
    ser.write((json.dumps(cmd_pid) + "\n").encode('utf-8'))
    time.sleep(0.05)
    
    # 2. Command Move
    cmd_move = {"cmd": "move", "distance": 12800, "accel": accel, "speed": 10000}
    ser.write((json.dumps(cmd_move) + "\n").encode('utf-8'))
    
    start_time = time.time()
    buf = bytearray()
    
    max_overshoot = 0
    total_error = 0
    samples = 0

    while True:
        if time.time() - start_time > 6.0:
            return None
            
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
                    pkt = buf[:TELE_PACKET_LEN]
                    
                    chk = 0
                    for b in pkt[2:28]: chk ^= b
                    if chk == pkt[28]:
                        fields = struct.unpack(TELE_FMT, bytes(pkt[2:]))
                        ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, _ = fields
                        
                        error = abs(target - meas)
                        total_error += error
                        samples += 1
                        
                        # Overshoot is distance past target. Assuming a positive move from 0.
                        # target can be up to 12800.
                        # Wait, we are doing relative moves, so target increases.
                        # It's better to just wait until p_dist is 0.
                        if p_dist == 0 and meas > target:
                            os = meas - target
                            if os > max_overshoot: max_overshoot = os
                    
                    buf = buf[TELE_PACKET_LEN:]
                    
                elif buf[1] == STOP_HEADER[1]:
                    if len(buf) < STOP_PACKET_LEN: break
                    pkt = buf[:STOP_PACKET_LEN]
                    pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                    reason = reason_b.rstrip(b'\x00').decode("ascii", errors="replace")
                    
                    duration = time.time() - start_time
                    avg_error = total_error / samples if samples > 0 else 0
                    return {
                        "duration": duration,
                        "overshoot": max_overshoot,
                        "avg_error": avg_error,
                        "reason": reason
                    }
                else:
                    buf = buf[1:]
        except Exception as e:
            print(f"Error during move: {e}")
            break
            
    return None

def main():
    parser = argparse.ArgumentParser(description="PD-Stepper AutoTune PID")
    parser.add_argument("--port", required=True, help="Serial port")
    args = parser.parse_args()

    print(f"[USB] Opening {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        print(f"[USB] ERROR: {e}")
        return

    print("--- Starting PID Auto-Tune ---")
    
    kp_range = [1.0, 1.5, 2.0]
    ki_range = [0.01, 0.03, 0.05]
    accel_range = [5000, 7000, 9000]

    best_score = float('inf')
    best_params = None

    for accel in accel_range:
        for kp in kp_range:
            for ki in ki_range:
                print(f"Testing Kp={kp}, Ki={ki}, Accel={accel} ...", end="", flush=True)
                
                # move 1
                res = run_test_move(ser, kp, ki, accel)
                if not res:
                    print(" Failed.")
                    continue
                
                # Move back
                cmd_move = {"cmd": "move", "distance": -12800, "accel": accel, "speed": 10000}
                try:
                    ser.write((json.dumps(cmd_move) + "\n").encode('utf-8'))
                except Exception as e:
                    print(f" USB write failed: {e}")
                    break
                
                # Wait for stop packet of the reverse move
                start_w = time.time()
                while time.time() - start_w < 5:
                    try:
                        if ser.in_waiting:
                            chunk = ser.read(ser.in_waiting)
                            if b"\xaa\xcc" in chunk: break
                        else:
                            time.sleep(0.01)
                    except Exception:
                        break
                
                score = (res['overshoot'] * 10) + res['avg_error'] + (res['duration'] * 100)
                
                # Penalize lag faults or physical estops
                if "Fault" in res['reason'] or "E-STOP" in res['reason']:
                    score += 10000
                    
                print(f" Score: {score:.2f} (OS: {res['overshoot']}, AvgErr: {res['avg_error']:.2f}, Dur: {res['duration']:.2f}s, {res['reason']})")
                
                if score < best_score:
                    best_score = score
                    best_params = (kp, ki, accel)

                time.sleep(0.5)

    print("\n=== Auto-Tune Results ===")
    if best_params:
        print(f"Best Kp: {best_params[0]}")
        print(f"Best Ki: {best_params[1]}")
        print(f"Best Accel: {best_params[2]}")
        print(f"Best Score: {best_score:.2f}")
    else:
        print("No successful tests.")

if __name__ == "__main__":
    main()
