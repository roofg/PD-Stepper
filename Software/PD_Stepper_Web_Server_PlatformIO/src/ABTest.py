import asyncio
import websockets
import json
import time

ESP32_IP = "192.168.4.4"
WS_URL = f"ws://{ESP32_IP}/ws"

async def run_move(telemetry_on):
    mode = "ON" if telemetry_on else "OFF"
    print(f"\n--- Starting Move with Telemetry {mode} ---")
    
    try:
        async with websockets.connect(WS_URL, open_timeout=5) as websocket:
            # 1. Toggle Telemetry
            await websocket.send(json.dumps({"cmd": "telemetry", "enabled": telemetry_on}))
            
            # 2. Command Move
            cmd = {"cmd": "move", "distance": 25600, "accel": 9000, "speed": 10000}
            await websocket.send(json.dumps(cmd))
            
            start_time = time.time()
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    
                    if data.get("type") == "telemetry":
                        if telemetry_on:
                            # print(".", end="", flush=True) # Minimal output to avoid Python-side jitter
                            pass
                    elif data.get("type") == "stop":
                        duration = time.time() - start_time
                        print(f"\nMove finished in {duration:.2f}s. Reason: {data.get('reason')}")
                        break
                except Exception as e:
                    print(f"Error during move: {e}")
                    break
    except Exception as e:
        print(f"Failed to connect: {e}")

async def main():
    print("PD-Stepper A/B Test: Motor Smoothness Comparison")
    
    # Test A: Telemetry ON
    await run_move(True)
    
    print("\nWaiting 2 seconds for motor to cool/settle...")
    await asyncio.sleep(2)
    
    # Test B: Telemetry OFF
    await run_move(False)

if __name__ == "__main__":
    asyncio.run(main())
