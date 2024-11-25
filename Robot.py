import numpy as np
import matplotlib.pyplot as plt

from Trajectories import QuinticPolynomialTrajectory

class Robot:
    def __init__(self,
                 arm_link_lengths,
                 joint_limits=None,
                 joint_max_vels=None,
                 joint_max_accs=None):
        # Robot has swing (base, J0),  lift (vertical Z axis, J1), elbow (J2), wrist (J3),
        # Gripper bool (end effector, J4)
        # Implement joint max speeds and accelerations
        # Implement trajectories
        # Base has 4 DOF - mounted on a vehicle with X, Y, Z coordinates and Z rotation
        # Move the origin while maintaining end effector position
        self.arm_link_lengths = arm_link_lengths
        self.n_arm_links = len(arm_link_lengths)
        self.n_joints = 5
        self.joint_states = [0.0] * self.n_joints
        self.gripper_vertical_offset = 0.5  # 0.5m below the lift and J1 origin
        self.base_origin = [0, 0, 0]
        self.trajectory_generator = QuinticPolynomialTrajectory()


    # TODO wrap the angles to pi and -pi
    # TODO implement joint limits in the config

    def generate_trajectory(self, target_joint_states, control_type='fk'):
        traj = []
        for joint_idx, target_joint_state in enumerate(target_joint_states):
            if control_type == 'fk':
                self.trajectory_generator.setup_trajectory(waypoints=[self.joint_states[joint_idx], target_joint_state])
                t_values, setpoints = self.trajectory_generator.solve_traj()
                # TODO analyse the trajectory so find v(t) and if its too high at any point, then we increase traj_time
                # and run again, do it inside the generator so we have a vel_limits etc in the setup traj and maybe solve_traj
                # has a respect_limits=True
                traj.append([t_values, setpoints])
        return traj

    def move_forward_kinematics(self, target_joint_states):
        if len(target_joint_states) > self.n_joints:
            raise ValueError(f"Too many joint states ({len(target_joint_states)} for robot with {self.n_joints} joints")

        a = 1
        for joint_idx, target_state in enumerate(target_joint_states):
            self.joint_states[joint_idx] = target_state
            # TODO implement the trajectories

    def move_inverse_kinematics(self, target_position: tuple[float, float, float], target_orientation: float=None):
        # TODO implement trajectories
        theta_1, theta_2, theta_3 = self.calculate_inverse_kinematics(target_position, target_orientation)
        self.joint_states[0] = theta_1
        self.joint_states[1] = target_position[2] + self.gripper_vertical_offset
        self.joint_states[2] = theta_2
        self.joint_states[3] = theta_3

    def calculate_inverse_kinematics(self, target_position: tuple[float, float, float], target_orientation: float=None):
        # Check target validity
        # TODO allow non origin based commands
        target_distance = np.linalg.norm(target_position)
        if target_distance > sum(self.arm_link_lengths):
            # TODO check if target too clsoe too?
            raise ValueError(f"Invalid IK target {target_position} with link lengths {self.arm_link_lengths}")

        # Now set the base and elbow joints to set the wrist centre
        # If no target_orientation is given, assume 0 degree approach (approach from left to right)
        wrist_length = self.arm_link_lengths[-1]
        if target_orientation is None:
            target_orientation = np.pi/2

        wrist_centre_point = [target_position[0] - (wrist_length * np.cos(target_orientation)),
                              target_position[1] - (wrist_length * np.sin(target_orientation)),
                              target_position[2]]
        wrist_centre_distance = np.linalg.norm([wrist_centre_point[0], wrist_centre_point[1]])

        print(f"Wrist centre: {wrist_centre_point}")
        # wrist centre position kinematics:
        # using cosine rule
        cos_theta_2 = (wrist_centre_distance ** 2 - self.arm_link_lengths[0] ** 2 - self.arm_link_lengths[1] ** 2) / (2 * self.arm_link_lengths[0] * self.arm_link_lengths[1])
        theta_2 = np.arccos(cos_theta_2)
        angle_to_wrist = np.arctan2(wrist_centre_point[1], wrist_centre_point[0])
        angle = np.arccos((self.arm_link_lengths[0]**2 + wrist_centre_distance**2 - self.arm_link_lengths[1]**2) / (2 * self.arm_link_lengths[0] * wrist_centre_distance))
        theta_1 = angle_to_wrist - angle

        ## This is the wrist joint
        theta_3 = target_orientation - (theta_1 + theta_2)

        # base, elbow, wrist
        return [theta_1, theta_2, theta_3]

    def move_base(self, new_position: tuple[float, float, float], maintain_ee: bool=True):
        pass

    def actuate_gripper(self, desired_state: int):
        # 0 = open
        # 1 = closed
        self.joint_states[-1] = desired_state


    def plot_state(self):
        fig, ax = plt.subplots()
        base = plt.Circle((0.0, 0.0), 0.1)
        ax.add_artist(base)

        j2_position = [self.arm_link_lengths[0] * np.cos(self.joint_states[0]), self.arm_link_lengths[0] * np.sin(self.joint_states[0])]
        j3_position = [j2_position[0] + self.arm_link_lengths[1] * np.cos(self.joint_states[0] + self.joint_states[2]),
                       j2_position[1] + self.arm_link_lengths[1] * np.sin(self.joint_states[0] + self.joint_states[2]),]
        j4_position = [j3_position[0] + self.arm_link_lengths[2] * np.cos(self.joint_states[0] + self.joint_states[2] + self.joint_states[3]),
                       j3_position[1] + self.arm_link_lengths[2] * np.sin(self.joint_states[0] + self.joint_states[2] + self.joint_states[3])]

        print(f"J2 position (Elbow): {j2_position}")
        print(f"J3 position (Wrist): {j3_position}")
        print(f"J4 position (End Effector): {j4_position}")

        plt.plot([0, j2_position[0]], [0, j2_position[1]], 'bo-', label='Base Offset (L1)')  # L1
        plt.plot([j2_position[0], j3_position[0]], [j2_position[1], j3_position[1]], 'go-', label='Elbow (L2)')  # L2
        plt.plot([j3_position[0], j4_position[0]], [j3_position[1], j4_position[1]], 'ro-', label='Wrist (L3)')  # L3
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0.5)
        plt.axis('square')
        plt.legend()
        plt.show()



if __name__ == '__main__':
    robot = Robot([0.1, 0.3, 0.1])
    # robot.move_forward_kinematics([0.0,0.0,np.pi/2,np.pi/2,0.0])
    robot.move_inverse_kinematics((0.3, 0.3, 0.0))
    robot.plot_state()

