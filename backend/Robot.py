import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
from Trajectories import QuinticPolynomialTrajectory

class Joints(Enum):
    BASE = 0
    LIFT = 1
    ELBOW = 2
    WRIST = 3
    GRIPPER = 4


class Robot:
    def __init__(self,
                 arm_link_lengths,
                 joint_limits=None,
                 joint_max_vels=None,
                 joint_max_accs=None):
        self.arm_link_lengths = arm_link_lengths
        self.n_arm_links = len(arm_link_lengths)
        self.n_joints = 5
        self.joint_states = [0.0] * self.n_joints
        self.wrist_vertical_offset = 0.5 # 0.2m below the elbow
        self.gripper_vertical_offset = 0.2  # 0.5m below the lift and J1 origin
        self.base_position = [0.0, 0.0, 0.0]
        self.trajectory_generator = QuinticPolynomialTrajectory()
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

        for joint_idx, target_state in enumerate(target_joint_states):
            self.joint_states[joint_idx] = target_state
            # TODO implement the trajectories
            # TODO save animation frames somewhere, and then the client requests a frame update,
            # update robot state and then frontend displays the new state

    def move_inverse_kinematics(self, target_position: tuple[float, float, float], target_orientation: float=None):
        # TODO implement trajectories
        theta_1, theta_2, theta_3 = self.calculate_inverse_kinematics(target_position[0:2], target_orientation)
        self.joint_states[Joints.BASE.value] = theta_1
        self.joint_states[Joints.LIFT.value] = target_position[2] + self.wrist_vertical_offset + self.gripper_vertical_offset
        self.joint_states[Joints.ELBOW.value] = theta_2
        self.joint_states[Joints.WRIST.value] = theta_3

    def calculate_inverse_kinematics(self, target_position: tuple[float, float], target_orientation: float=None):
        # Check target validity
        relative_target = np.subtract(target_position, self.base_position[0:2])
        target_distance = np.linalg.norm(relative_target)
        if target_distance > sum(self.arm_link_lengths):
            # TODO check if target is too close to base
            raise ValueError(f"Invalid IK target {target_position} with link lengths {self.arm_link_lengths}")

        # Now set the base and elbow joints to set the wrist centre
        # If no target_orientation is given, assume 0 degree approach (approach from left to right)
        wrist_length = self.arm_link_lengths[-1]
        if target_orientation is None:
            target_orientation = np.pi/2

        wrist_centre_point = [relative_target[0] - (wrist_length * np.cos(target_orientation)),
                              relative_target[1] - (wrist_length * np.sin(target_orientation))]
        wrist_centre_distance = np.linalg.norm(wrist_centre_point)

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
        return theta_1, theta_2, theta_3

    def get_joint_positions(self):
        base = self.base_position
        vertical_lift = [self.base_position[0], self.base_position[1], self.joint_states[Joints.LIFT.value]]
        elbow = [
            self.base_position[0] + (self.arm_link_lengths[0] * np.cos(self.joint_states[Joints.BASE.value])),
            self.base_position[1] + (self.arm_link_lengths[0] * np.sin(self.joint_states[Joints.BASE.value])),
            self.joint_states[Joints.LIFT.value]
        ]

        wrist = [
            elbow[0] + (self.arm_link_lengths[1] * np.cos(self.joint_states[Joints.BASE.value] + self.joint_states[Joints.ELBOW.value])),
            elbow[1] + (self.arm_link_lengths[1] * np.sin(self.joint_states[Joints.BASE.value] + self.joint_states[Joints.ELBOW.value])),
            (self.joint_states[Joints.LIFT.value] - self.wrist_vertical_offset)
        ]

        ee = [
            wrist[0] + self.arm_link_lengths[2] * np.cos(
                self.joint_states[Joints.BASE.value] + self.joint_states[Joints.ELBOW.value] + self.joint_states[Joints.WRIST.value]
            ),
            wrist[1] + self.arm_link_lengths[2] * np.sin(
                self.joint_states[Joints.BASE.value] + self.joint_states[Joints.ELBOW.value] + self.joint_states[Joints.WRIST.value]
            ),
            (self.joint_states[Joints.LIFT.value] - self.wrist_vertical_offset - self.gripper_vertical_offset)
        ]

        return base, vertical_lift, elbow, wrist, ee

    def get_arm_joint_positions(self):
        *_, elbow, wrist, ee = self.get_joint_positions()
        return elbow, wrist, ee

    def get_joint_states(self):
        return self.joint_states


    def get_ee_position(self):
        # Get ee position relative to the base
        *_, ee = self.get_arm_joint_positions()
        return ee


    def move_base(self, new_position: tuple[float, float], maintain_ee: bool=True):
        # Assume base only moves in a planar motion
        if maintain_ee:
            ee_ik_target = self.get_ee_position()
            target_orientation = self.joint_states[0] + self.joint_states[2] + self.joint_states[3]
        self.base_position[0] = new_position[0]
        self.base_position[1] = new_position[1]
        if maintain_ee:
            self.move_inverse_kinematics(ee_ik_target, target_orientation)


    def actuate_gripper(self, desired_state: int):
        # 0 = open
        # 1 = closed
        self.joint_states[-1] = desired_state

    def check_lengths(self):
        elbow, wrist, ee = self.get_arm_joint_positions()
        base_length = np.linalg.norm(np.subtract(self.base_position[0:2], elbow[0:2]))
        elbow_length = np.linalg.norm(np.subtract(elbow[0:2], wrist[0:2]))
        wrist_length = np.linalg.norm(np.subtract(wrist[0:2], ee[0:2]))

        print(f"Base length is: {base_length}")
        print(f"elbow length is: {elbow_length}")
        print(f"wrist length is: {wrist_length}")

    def plot_state(self):
        fig, ax = plt.subplots()
        base = plt.Circle((self.base_position[0], self.base_position[1]), 0.1)
        ax.add_artist(base)

        elbow, wrist, ee = self.get_arm_joint_positions()

        print(f"J2 position (Elbow): {elbow}")
        print(f"J3 position (Wrist): {wrist}")
        print(f"J4 position (End Effector): {ee}")

        self.check_lengths()

        plt.plot([self.base_position[0], elbow[0]], [self.base_position[1], elbow[1]], 'bo-', label='Base Offset (L1)')  # L1
        plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'go-', label='Elbow (L2)')  # L2
        plt.plot([wrist[0], ee[0]], [wrist[1], ee[1]], 'ro-', label='Wrist (L3)')  # L3
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0.5)
        ax.axis('equal')
        plt.tight_layout()
        plt.legend()
        plt.show()


if __name__ == '__main__':
    robot = Robot([0.3, 0.6, 0.2])
    robot.move_inverse_kinematics((0.6, 0.2, 0.8))
    # robot.move_forward_kinematics([0.0, 0.0, np.pi/2, np.pi/2, 0.0])
    robot.plot_state()
    robot.move_base([0.2, 0.0], True)
    robot.plot_state()

