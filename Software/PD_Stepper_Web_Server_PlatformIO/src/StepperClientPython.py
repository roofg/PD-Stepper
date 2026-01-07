import asyncio
import websockets
import json

async def move():
    async with websockets.connect("ws://192.168.4.4/ws") as websocket:
        # Move 1: Relative movement (1 full rotation at 32 microsteps)
        cmd1 = {"cmd": "move", "distance": 25600, "accel": 5000, "speed": 5000}
        await websocket.send(json.dumps(cmd1))
        print(f"Sent Relative Move: {cmd1}")
        
        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)
                if data.get("type") == "telemetry":
                    p = data.get('pos', 0)
                    m = data.get('meas', 0)
                    t = data.get('target', 0) # Added target variable
                    l = data.get('lag', 0)
                    v = data.get('vel', 0)
                    print(f"Pos: {p}, Meas: {m}, Tgt: {t}, Lag: {l}, Vel: {v}") # Modified print statement
                elif data.get("type") == "done":
                    print(f"Movement 1 Complete. Final Pos: {data['pos']}")
                    break
                elif data.get("type") == "stop":
                    print(f"STOPPED: {data.get('reason')} at Pos: {data.get('pos')}")
                    break
            except websockets.exceptions.ConnectionClosed:
                print("Connection closed") # need more information about the error
                return

        # Move 2: Absolute move back to home (0)
        #cmd2 = {"cmd": "move", "distance": 0, "accel": 1000, "speed": 5000, "abs": True}
        #await websocket.send(json.dumps(cmd2))
        #print(f"Sent Absolute Move to Home: {cmd2}")

asyncio.run(move())