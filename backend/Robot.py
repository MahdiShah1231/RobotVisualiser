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
                 joint_max_vels=None,
                 joint_max_accs=None,
                 base_max_vel=None,
                 base_max_acc=None):
        self.arm_link_lengths = arm_link_lengths
        self.n_arm_links = len(arm_link_lengths)
        self.n_joints = 4  # Base, lift, elbow, wrist
        self.joint_states = [0.0] * self.n_joints
        self.joint_max_vels = joint_max_vels
        self.joint_max_accs = joint_max_accs
        self.base_max_vel = base_max_vel
        self.base_max_acc = base_max_acc
        self.wrist_vertical_offset = 0.1 # 0.1m below the elbow
        self.gripper_vertical_offset = 0.1  # 0.1m below the lift and J1 origin
        self.base_position = [0.0, 0.0, 0.0]
        self.trajectory_points = {"base": [], "joints": []}
        self.trajectory_generator = QuinticPolynomialTrajectory()
    # TODO implement joint limits in the config
    # TODO have a robot config.yaml and load it in backend and frontend for consistency

    def generate_trajectory(self, target_joint_states, control_type='fk'):
        traj = []
        max_traj_length = 0
        for joint_idx, target_joint_state in enumerate(target_joint_states):
            if control_type == 'fk':
                self.trajectory_generator.setup_trajectory(
                    waypoints=[self.joint_states[joint_idx], target_joint_state],
                    vel_limit=self.joint_max_vels[joint_idx],
                    acc_limit=self.joint_max_accs[joint_idx]
                )
                _, setpoints = self.trajectory_generator.solve_traj(respect_limits=True)
                traj.append(setpoints)
                max_traj_length = max(max_traj_length, len(setpoints))  ## Need to find longest trajectory

        # Pad shorter trajectories with their last value
        padded_traj = []
        for joint_traj in traj:
            padded_traj.append(joint_traj + [joint_traj[-1]] * (max_traj_length - len(joint_traj)))

        traj = list(zip(*padded_traj))
        self.trajectory_points["joints"] = traj
        return traj

    def move_forward_kinematics(self, target_joint_states, animate=False):
        if len(target_joint_states) > self.n_joints:
            print(f"Error. Too many joint states ({len(target_joint_states)} for robot with {self.n_joints} joints")
            return

        if not animate:
            for joint_idx, target_state in enumerate(target_joint_states):
                self.joint_states[joint_idx] = target_state
        else:
            self.generate_trajectory(target_joint_states, 'fk')

    def move_inverse_kinematics(self, target_position: tuple[float, float, float], target_orientation: float=None, animate=False):
        # For IK trajectory simplicity, find joint angles at start and end, then generate polynomial traj between points in joint
        # space. Quicker than generating polynomial traj in cartesian space and calling IK on each point
        base_angle, elbow_angle, wrist_angle = self.calculate_inverse_kinematics(target_position[0:2], target_orientation)
        lift_state = target_position[2] + self.wrist_vertical_offset + self.gripper_vertical_offset
        if not animate:
            self.joint_states[Joints.BASE.value] = base_angle
            self.joint_states[Joints.LIFT.value] = lift_state
            self.joint_states[Joints.ELBOW.value] = elbow_angle
            self.joint_states[Joints.WRIST.value] = wrist_angle
        else:
            fk_target = [base_angle, lift_state, elbow_angle, wrist_angle]
            self.generate_trajectory(fk_target, 'fk')

    def calculate_inverse_kinematics(self, target_position: tuple[float, float], target_orientation: float=None, base_position=None):
        # Check target validity
        if base_position is None:
            base_position = self.base_position[0:2]
        else:
            base_position = base_position[0:2]
        relative_target = np.subtract(target_position, base_position)
        target_distance = np.around(np.linalg.norm(relative_target), 5)
        if target_distance > sum(self.arm_link_lengths):
            # TODO check if target is too close to base
            print(f"Error. Invalid IK target {target_position}, distance: {target_distance}, max reach: {sum(self.arm_link_lengths)}")
            return self.joint_states[0], self.joint_states[2], self.joint_states[3]

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
        cos_theta_2 = np.clip(cos_theta_2, -1.0, 1.0)
        theta_2 = np.arccos(cos_theta_2)
        angle_to_wrist = np.arctan2(wrist_centre_point[1], wrist_centre_point[0])
        cos_angle = (self.arm_link_lengths[0]**2 + wrist_centre_distance**2 - self.arm_link_lengths[1]**2) / (2 * self.arm_link_lengths[0] * wrist_centre_distance)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
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

    def execute_traj(self):
        base_traj_points = self.trajectory_points["base"]
        joint_traj_points = self.trajectory_points["joints"]
        if len(base_traj_points) > 0:
            self.base_position = base_traj_points.pop(0)
        if len(joint_traj_points) > 0:
            self.joint_states = joint_traj_points.pop(0)

    def get_robot_state(self):
        self.execute_traj()
        return self.get_base_position(), self.get_joint_states()


    def get_ee_position(self):
        # Get ee position relative to the base
        *_, ee = self.get_arm_joint_positions()
        ee = [np.around(val, 5) for val in ee]
        return ee

    def generate_base_traj(self, target_position, ee_target_position=None, ee_target_orientation=None):
        # Assuming base only moves on x,y for simplicity
        self.trajectory_generator.setup_trajectory(waypoints=(self.base_position[0], target_position[0]),
                                                   vel_limit=self.base_max_vel,
                                                   acc_limit=self.base_max_acc)
        t, x_traj = self.trajectory_generator.solve_traj(respect_limits=True)

        self.trajectory_generator.setup_trajectory(waypoints=(self.base_position[1], target_position[1]),
                                                   vel_limit=self.base_max_vel,
                                                   acc_limit=self.base_max_acc)
        t, y_traj = self.trajectory_generator.solve_traj(respect_limits=True)

        z_traj = [0.0] * len(y_traj)

        traj = list(zip(x_traj, y_traj, z_traj))
        instant_traj = []
        if ee_target_position:
            # For each base position, calculate the joint angles using IK to maintain EE position
            # For simplicity using instantaneous points
            for base_pos in traj:
                base_angle, elbow_angle, wrist_angle = self.calculate_inverse_kinematics(ee_target_position[0:2], ee_target_orientation, base_position=base_pos)
                lift_state = ee_target_position[2] + self.wrist_vertical_offset + self.gripper_vertical_offset
                instant_joint_targets = [base_angle, lift_state, elbow_angle, wrist_angle]
                instant_traj.append(instant_joint_targets)

        self.trajectory_points["base"] = traj
        self.trajectory_points["joints"] = instant_traj

    def move_base(self, new_position: tuple[float, float], maintain_ee: bool=False, animate=False):
        # Assume base only moves in a planar motion
        # For simplicity, the base has no orientation and thus can move in any direction without turning
        # TODO need to check base final position leaves ee in range of target pose
        if animate:
            ee_ik_target = None
            ee_ik_target_orientation = None
            if maintain_ee:
                ee_ik_target = self.get_ee_position()
                ee_ik_target_orientation = self.joint_states[0] + self.joint_states[2] + self.joint_states[3]
            self.generate_base_traj(new_position, ee_ik_target, ee_ik_target_orientation)
        else:
            self.base_position[0] = new_position[0]
            self.base_position[1] = new_position[1]
            self.base_position[2] = 0.0

    def get_base_position(self):
        return self.base_position

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

        # print(f"J2 position (Elbow): {elbow}")
        # print(f"J3 position (Wrist): {wrist}")
        # print(f"J4 position (End Effector): {ee}")

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
    robot.move_inverse_kinematics((0.6, 0.3, 0.8))
    # robot.move_base([5.5, 0.0], False)
    # robot.move_forward_kinematics([0.0, 0.0, np.pi/2, np.pi/2, 0.0])
    robot.plot_state()

