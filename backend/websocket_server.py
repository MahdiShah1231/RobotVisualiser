import asyncio
import websockets
from Robot import Robot
from functools import partial

connected_clients = set()
async def send_joint_states(robot):
    while True:
        joint_states = robot.get_joint_states()
        message = f"Joint States: {joint_states}"
        await asyncio.gather(*(client.send(message) for client in connected_clients))

        # 10 Hz broadcast
        # TODO experiment and change
        await asyncio.sleep(0.1)

async def websocket_handler(websocket, robot):
    print(f"Client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            command = message.split(" ")
            if command[0] == "fk":
                target_joint_states = list(map(float, command[1:]))
                print("Fk command received.")
                print(f"Target joint angles: {target_joint_states}")
                robot.move_forward_kinematics(target_joint_states)
                await websocket.send(f"FK command executed.")

            if command[0] == "ik":
                ik_command = command [1::]
                if len(ik_command) == 4:
                    target_orientation = float(ik_command[-1])
                    target_ee_position = list(map(float, ik_command[0:-1]))
                else:
                    target_orientation = None
                    target_ee_position = list(map(float, ik_command))

                print(f"Ik command received.")
                print(f"Target end effector position: {target_ee_position}")
                robot.move_inverse_kinematics(target_position=target_ee_position, target_orientation=target_orientation)
                await websocket.send(f"IK command executed.")

            if command[0] == "base":
                base_move_command = command[1::]
                if len(base_move_command) == 3:
                    target_base_position = list(map(float, base_move_command[0:2]))
                    maintain_ee = bool(base_move_command[-1])
                else:
                    target_base_position = list(map(float, base_move_command))
                    maintain_ee = True
                robot.move_base(new_position=target_base_position, maintain_ee=maintain_ee)
                await websocket.send(f"Move base command executed")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Client disconnected: {e}")
    finally:
        connected_clients.remove(websocket)

async def main():
    # TODO allow configuration of robot from front end
    robot = Robot([0.3, 0.6, 0.2])
    websocket_handler_bound = partial(websocket_handler, robot=robot)
    server = await websockets.serve(websocket_handler_bound, "0.0.0.0", 8000)
    print("WebSocket server started on ws://0.0.0.0:8000")
    await asyncio.gather(server.wait_closed(), send_joint_states(robot))


if __name__ == "__main__":
    asyncio.run(main())