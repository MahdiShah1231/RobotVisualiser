import asyncio
import websockets
import numpy as np

# Python websocket client to test backend functionality
async def client():
    uri = "ws://localhost:8000"
    async with websockets.connect(uri) as websocket:
        print("Connected to WebSocket server.")

        ## Wait a bit before sending command
        i = 0
        while i in range(5):
            message = await websocket.recv()
            print(f"Broadcast from server: {message}")
            i += 1

        ## Send FK command
        # await websocket.send(f"fk 0.0 0.0 {np.pi/2} {np.pi/2} 0.0")

        ## Send IK command
        # await websocket.send(f"ik 0.6 0.2 0.8 {np.pi/2}")

        ## Send move base
        # await websocket.send(f"base 0.0 0.1 true")

        response = await websocket.recv()
        print(f"Server response: {response}")

        while True:
            message = await websocket.recv()
            print(message)

asyncio.run(client())