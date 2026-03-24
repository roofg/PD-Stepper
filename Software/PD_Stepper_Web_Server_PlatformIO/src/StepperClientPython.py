import argparse
import struct
import serial
import json
import time

TELE_HEADER      = (0xAA, 0xBB)
STOP_HEADER      = (0xAA, 0xCC)
TELE_PACKET_LEN  = 33
STOP_PACKET_LEN  = 38
TELE_FMT         = "<IiiihhhhHBBh"  # ts, pos, meas, target, lag, vel, p_acc, p_dist, sg, cs, pwm, mvel
STOP_FMT         = "<i32s"       # int32, char[32]

# Encoder-count conversion constants (AS5600: 4096 counts/rev)
COUNTS_PER_REV   = 4096
DEG_PER_COUNT    = 360.0 / COUNTS_PER_REV
RPM_PER_CPS      = 60.0 / COUNTS_PER_REV

def enc_to_deg(counts): return counts * DEG_PER_COUNT
def enc_per_sec_to_rpm(cps): return cps * RPM_PER_CPS
def move(ser: serial.Serial):
    # Move 1: Relative movement (1 full rotation at 32 microsteps)
    cmd1 = {"cmd": "move", "distance": 25600, "accel": 7000, "speed": 9000}
    ser.write((json.dumps(cmd1) + "\n").encode('utf-8'))
    print(f"Sent Relative Move: {cmd1}")
    
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
                    pkt = buf[:TELE_PACKET_LEN]
                    
                    chk = 0
                    for b in pkt[2:32]: chk ^= b
                    if chk == pkt[32]:
                        fields = struct.unpack(TELE_FMT, bytes(pkt[2:32]))
                        ts, pos, meas, target, lag, vel, p_acc, p_dist, sg_result, _cs, _pwm, _mvel = fields
                        time_s = ts / 1000000.0
                        print(f"[{time_s:7.2f}s] P:{enc_to_deg(pos):7.1f}° M:{enc_to_deg(meas):7.1f}° T:{enc_to_deg(target):7.1f}° L:{enc_to_deg(lag):5.2f}° V:{enc_per_sec_to_rpm(vel):6.1f}RPM SG:{sg_result:4}")
                    
                    buf = buf[TELE_PACKET_LEN:]
                    
                elif buf[1] == STOP_HEADER[1]:
                    if len(buf) < STOP_PACKET_LEN: break
                    pkt = buf[:STOP_PACKET_LEN]
                    pos, reason_b = struct.unpack(STOP_FMT, bytes(pkt[2:]))
                    reason = reason_b.rstrip(b'\x00').decode("ascii", errors="replace")
                    
                    print(f"\nSTOPPED: {reason} at Pos: {enc_to_deg(pos):.1f}°")
                    if reason == "Lag Fault":
                        print("DEBUG: 'Lag Fault' means the motor could not keep up with the trajectory.")
                        print("       Check for mechanical binding, insufficient current, or too high acceleration/speed.")
                    elif reason == "E-STOP (SW1)":
                        print("DEBUG: Physical Emergency Stop button (SW1) was pressed.")
                    return
                else:
                    buf = buf[1:]
                    
        except Exception as e:
            print(f"\nError: {e}")
            return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PD-Stepper Python Client")
    parser.add_argument("--port", required=True, help="Serial port")
    args = parser.parse_args()

    print(f"Opening {args.port} at 921600 baud...")
    try:
        ser = serial.Serial(args.port, 921600, timeout=0.1)
    except Exception as e:
        print(f"ERROR: {e}")
        exit(1)

    print("Connected successfully!")
    time.sleep(1)
    try:
        move(ser)
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
