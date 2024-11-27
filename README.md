# Robot Visualiser
## Usage
Fist setup the python environment by cding to the directory and using `pipenv install`. Then enter the environment using
`pipenv shell`.

### Testing backend (without WebSocket)
After setting up the python shell, run `python Robot.py`, after configuring the commands you want to send
at the bottom of the file.

### Testing backend (WebSocket)
Run `python websocket_server.py`  
In another terminal, run `python websocket_client.py`, after configuring the commands you want to send
in the client file.

## Backend
Features IK and FK control for the mobile platform. It can be tested by setting up
the python pipenv environment and then just running the Robot.py file.

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
Frontend is not working and has not been finished due to time constraints unfortunately.  
Currently the basic layout of the web page is setup, with a simple render of the robot state,
as well as an interface to allow IK and FK commands to be sent to the backend websocket Server (works)