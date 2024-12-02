from abc import ABC, abstractmethod

import numpy
import numpy as np


class TrajectoryBase(ABC):
    """Abstract Base Class for a trajectory generator."""

    def __init__(self) -> None:
        """Initialise the base trajectory generator."""
        self._waypoints = ()
        self._traj_time = None
        self._frequency = None
        self.vel_limit = None
        self.acc_limit = None
        self._t_values = []
        self._setpoints = []

    def setup_trajectory(self,
                         waypoints: tuple[float, float],
                         time: float = 3.0,
                         vel_limit: float = None,
                         acc_limit: float = None,
                         frequency: int = 60) -> None:
        """Set up the trajectory waypoints.

        Args:
            waypoints: The trajectory waypoints. Start and End value for single coordinate/joint angle (one at a time).
            time: The time taken to complete the trajectory.
            frequency: The sampling frequency of the trajectory points.
        """
        self._clear_traj()  # First clear any existing solutions
        self._waypoints = waypoints
        self.vel_limit = vel_limit
        self.acc_limit = acc_limit
        self._traj_time = time
        self._frequency = frequency

    @abstractmethod
    def solve_traj(self) -> tuple[numpy.ndarray, list[float]]:
        """Abstract method to be implemented."""
        raise NotImplementedError

    def _clear_traj(self) -> None:
        """Clear the saved trajectory."""
        self._t_values = []
        self._setpoints = []

    def check_limits(self):
        pass


class QuinticPolynomialTrajectory(TrajectoryBase):
    """A derived trajectory generator to create a quintic polynomial trajectory.

    Used to solve trajectories with 6 constraints on starting and final positions, velocities and accelerations.

    """
    # a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5 = final_position
    # 0a0 + a1 + 2a2t + 3a3t^2 + 4a4t^3 + 5a5t^4 = final_velocity = 0
    # 0a0 + 0a1 + 2a2 + 6a3t + 12a4t^2 + 20a5t^3 = final_acceleration = 0
    # a0 + 0 + 0 + 0 + 0 + 0 = initial position
    # 0 + a1 + 0 + 0 + 0 + 0 = initial velocity = 0
    # 0 + 0 + 2a2 + 0 + 0 + 0 = initial_acceleration = 0
    # Ax = b

    def __init__(self) -> None:
        """Initialise the quintic polynomial generator."""
        super().__init__()

    def solve_traj(self, respect_limits=False) -> tuple[numpy.ndarray[float], list[float]]:
        """Solve the quintic polynomial trajectory.

        Returns:
            A tuple containing the sampled time points and the corresponding setpoints.
        """
        # Ax = b, solving for x (matrix of coefficients a0 -> a5) to give trajectory going from initial setpoint
        # to final setpoint in t = traj_time
        found = False
        while not found:
            setpoints = []
            a_matrix = np.array([[1, self._traj_time, self._traj_time ** 2, self._traj_time ** 3, self._traj_time ** 4, self._traj_time ** 5],
                                 [0, 1, 2 * self._traj_time, 3 * (self._traj_time ** 2), 4 * (self._traj_time ** 3), 5 * (self._traj_time ** 4)],
                                 [0, 0, 0, 6 * self._traj_time, 12 * (self._traj_time ** 2), 20 * (self._traj_time ** 3)],
                                 [1, 0, 0, 0, 0, 0],
                                 [0, 1, 0, 0, 0, 0],
                                 [0, 0, 2, 0, 0, 0]])

            b_matrix = np.array([self._waypoints[1], 0, 0, self._waypoints[0], 0, 0])

            x_matrix = np.linalg.solve(a_matrix, b_matrix)

            # Now need to evaluate the coefficients for 0 < t < traj_time at the increments specified for the whole trajectory
            # a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5 = final_position for t = traj time, = intermediate position for t < traj_time

            t_values = np.linspace(0, self._traj_time, int(self._frequency * self._traj_time))

            for t in t_values:
                setpoint = x_matrix[0] + x_matrix[1] * t + x_matrix[2] * (t**2) + x_matrix[3] * (t**3) + x_matrix[4] * (t**4) + x_matrix[5] * (t**5)
                setpoints.append(setpoint)

            if respect_limits:
                violates_vel_limit = violates_acc_limit = False
                if self.vel_limit is not None:
                    velocities = [x_matrix[1] + 2 * x_matrix[2] * t + 3 * x_matrix[3] * t ** 2 + 4 * x_matrix[4] * t ** 3 + 5 * x_matrix[5] * t ** 4 for t in t_values]
                    violates_vel_limit = max(abs(v) for v in velocities) > self.vel_limit
                if self.acc_limit is not None:
                    accelerations = [2 * x_matrix[2] + 6 * x_matrix[3] * t + 12 * x_matrix[4] * t ** 2 + 20 * x_matrix[5] * t ** 3 for t in t_values]
                    violates_acc_limit = max(abs(a) for a in accelerations) > self.acc_limit

                if violates_vel_limit or violates_acc_limit:
                    ## Extend trajectory time by 10%
                    self._traj_time *= 1.1
                    continue

            self._t_values = t_values
            self._setpoints = setpoints
            found = True

        return t_values, setpoints
