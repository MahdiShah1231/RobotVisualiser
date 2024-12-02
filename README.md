# Robot Visualiser
## Usage
Fist setup the python environment by cding to the directory and using `pipenv install`. Then enter the environment using
`pipenv shell`.

### Testing backend (without WebSocket)
After setting up the python shell, run `python Robot.py`, after configuring the commands you want to send
at the bottom of the file.

### Testing backend (WebSocket)
Run `python websocket_server.py`, configure the robot parameters at the bottom.
#### Using python client
In another terminal, run `python websocket_client.py`, after configuring the commands you want to send
in the client file.
#### Using Web UI
Open a terminal to where the front end folder directory is, and run `npm run dev`, then open browser
and go to http://localhost:3000

## Backend
Features IK and FK control for the mobile platform. Uses a python websocket server
to communicate with the frontend to display the robot state on the 3D web UI.

### Key Functions
#### move_inverse_kinematics(target_position, target_orientation)
This function allows you to send an IK command to the robot.  
The target_position is a list or tuple in the form [x,y,z].  
The target_orientation is a float describing the approach angle of the end effector. pi/2 = from below,
0 = from left to right..

      
#### move_forward_kinematics(target_joint_states)
This function sends an FK command. The targets must be sent in order.  
The target_joint_states is a list in the form [base_target, lift_target, elbow_target, wrist_target, gripper_target].


#### move_base(new_position, maintain_ee)
Moves the base.

#### plot_state()
Plots the current robot state. Call this after sending some of the above commands
to see how the robot state changes. It is not animated with trajectories.

## Frontend
Basic UI to allow interaction with the backend to send joint space (FK) and cartesian space (IK) commands to the robot.
Can also send commands to move the mobile base.