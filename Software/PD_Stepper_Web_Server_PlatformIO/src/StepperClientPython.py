import asyncio
import websockets
import json
import sys
import socket

# Configuration
ESP32_IP = "192.168.4.4"
WS_URL = f"ws://{ESP32_IP}/ws"

async def move():
    print(f"Connecting to {WS_URL}...")
    try:
        async with websockets.connect(WS_URL, open_timeout=5) as websocket:
            print("Connected successfully!")
            
            # Move 1: Relative movement (1 full rotation at 32 microsteps)
            cmd1 = {"cmd": "move", "distance": 25600, "accel": 7000, "speed": 9000}
            await websocket.send(json.dumps(cmd1))
            print(f"Sent Relative Move: {cmd1}")
            
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    
                    msg_type = data.get("type")
                    
                    if msg_type == "telemetry":
                        time_s = data.get('time', 0) / 1000000.0 # micros to seconds
                        print(f"[{time_s:7.2f}s] P:{data.get('pos'):6} M:{data.get('meas'):6} T:{data.get('target'):6} L:{data.get('lag'):4} V:{data.get('vel'):5}")
                    
                    elif msg_type == "done":
                        # Legacy support or future proofing
                        print(f"\nMovement Complete. Final Pos: {data.get('pos')}")
                        break
                        
                    elif msg_type == "stop":
                        reason = data.get('reason', 'Unknown')
                        pos = data.get('pos', 0)
                        print(f"\nSTOPPED: {reason} at Pos: {pos}")
                        if reason == "Lag Fault":
                            print("DEBUG: 'Lag Fault' means the motor could not keep up with the trajectory.")
                            print("       Check for mechanical binding, insufficient current, or too high acceleration/speed.")
                        elif reason == "E-STOP (SW1)":
                            print("DEBUG: Physical Emergency Stop button (SW1) was pressed.")
                        break
                        
                except websockets.exceptions.ConnectionClosed as e:
                    print(f"\nConnection closed unexpectedly: {e}")
                    print("DEBUG: The ESP32 might have rebooted, crashed, or the Wi-Fi connection was lost.")
                    return
                except json.JSONDecodeError:
                    print(f"Received malformed JSON: {message}")

    except (socket.timeout, asyncio.TimeoutError):
        print(f"Error: Connection timed out to {ESP32_IP}")
        print("DEBUG: Make sure you are connected to the 'PD Stepper' Wi-Fi access point.")
    except ConnectionRefusedError:
        print(f"Error: Connection refused by {ESP32_IP}")
        print("DEBUG: The ESP32 is reachable, but the WebSocket server is not responding.")
    except Exception as e:
        print(f"An unexpected error occurred: {type(e).__name__}: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(move())
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
