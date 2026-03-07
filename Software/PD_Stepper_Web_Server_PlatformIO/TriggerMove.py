import asyncio
import websockets
import json

async def trigger():
    try:
        async with websockets.connect('ws://192.168.4.4/ws') as ws:
            await ws.send(json.dumps({'cmd': 'move', 'distance': 51200, 'accel': 9000, 'speed': 12000.0}))
            print("Move command sent.")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(trigger())
