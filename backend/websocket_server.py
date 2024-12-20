import asyncio
import websockets
from Robot import Robot
from functools import partial
import numpy as np

connected_clients = set()
async def send_robot_states(robot):
    while True:
        base_position, joint_states = robot.get_robot_state()
        joint_states = ",".join(map(str,joint_states))
        base_position = ",".join(map(str, base_position))
        message = f"JointStates: {joint_states};BasePosition: {base_position}"
        await asyncio.gather(*(client.send(message) for client in connected_clients))

        # 50 Hz broadcast
        # TODO experiment and change
        await asyncio.sleep(0.02)

async def websocket_handler(websocket, robot):
    print(f"Client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            print(f"Received message: {message}")
            command = message.split(" ")
            command_type = command[0]
            if command_type == "fk":
                target_joint_states = list(map(float, command[1:]))
                print("Fk command received.")
                print(f"Target joint angles: {target_joint_states}")
                robot.move_forward_kinematics(target_joint_states=target_joint_states, animate=True)
            elif command_type == "ik":
                ik_command = command [1::]
                if len(ik_command) == 4:
                    target_orientation = float(ik_command[-1])
                    target_ee_position = list(map(float, ik_command[0:-1]))
                else:
                    target_orientation = None
                    target_ee_position = list(map(float, ik_command))

                print(f"Ik command received.")
                print(f"Target end effector position: {target_ee_position}")
                robot.move_inverse_kinematics(target_position=target_ee_position, target_orientation=target_orientation, animate=True)
            elif command_type == "base":
                base_move_command = command[1::]
                target_base_position = list(map(float, base_move_command[0:2]))
                maintain_ee = base_move_command[-1] == "true"

                print(f"Move base command received.")
                print(f"Target base position: {target_base_position}")
                robot.move_base(new_position=target_base_position, maintain_ee=maintain_ee, animate=True)
            else:
                print(f"Unrecognised command: {command_type}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Client disconnected: {e}")
    finally:
        connected_clients.remove(websocket)

async def main():
    # TODO allow configuration of robot from front end
    robot = Robot(arm_link_lengths=[0.3, 0.6, 0.2],
                  joint_max_vels=[np.pi/3, 0.5, np.pi/3, np.pi/3],
                  joint_max_accs=[1.0, 1.0, 1.0, 1.0])
    websocket_handler_bound = partial(websocket_handler, robot=robot)
    server = await websockets.serve(websocket_handler_bound, "0.0.0.0", 8000)
    print("WebSocket server started on ws://0.0.0.0:8000")
    await asyncio.gather(server.wait_closed(), send_robot_states(robot))


if __name__ == "__main__":
    asyncio.run(main())