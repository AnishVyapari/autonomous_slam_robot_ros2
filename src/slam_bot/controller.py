#!/usr/bin/env python3
"""
Robot Controller Module

Provides proportional navigation and PID control utilities for a differential-drive robot.
Students can extend this with trajectory following, dynamic window approach, or MPC.

Author: Autonomous SLAM Robot ROS2 Project
License: MIT
"""
from typing import Tuple, Optional
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose


class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, i_max: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.i_max = i_max
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float) -> float:
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.i_max, self.i_max)
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class RobotController:
    """
    Simple reactive navigator using obstacle distances and PID heading control.
    """
    def __init__(self, node, max_linear_vel: float = 0.5, max_angular_vel: float = 1.0):
        self.node = node
        self.max_linear = max_linear_vel
        self.max_angular = max_angular_vel

        # PID for heading error (radians)
        control_dt = 1.0 / float(node.get_parameter('control_frequency').value)
        self.heading_pid = PID(kp=1.2, ki=0.0, kd=0.1, dt=control_dt)

        # Safety distances (meters)
        self.safe_front = 0.5
        self.safe_side = 0.35

    def compute_velocity(self, pose: Optional[Pose], scan: Optional[LaserScan], obstacle: dict) -> Tuple[float, float]:
        """
        Compute linear and angular velocity commands.
        - If obstacle too close in front, turn away based on which side is clearer
        - Otherwise, move forward with slight wall-following behavior
        """
        if obstacle is None or scan is None:
            return 0.0, 0.0

        front = obstacle.get('front', np.inf)
        left = obstacle.get('left', np.inf)
        right = obstacle.get('right', np.inf)

        linear = self.max_linear
        yaw_error = 0.0

        # Obstacle avoidance
        if front < self.safe_front:
            linear = 0.0
            # Turn toward the side with more space
            yaw_error = 0.6 if left > right else -0.6
        else:
            # Gentle bias to keep distance from nearest wall
            if left < self.safe_side:
                yaw_error = -0.3  # steer right
            elif right < self.safe_side:
                yaw_error = 0.3   # steer left

        # PID for angular velocity
        angular = float(np.clip(self.heading_pid.compute(yaw_error), -self.max_angular, self.max_angular))
        linear = float(np.clip(linear, 0.0, self.max_linear))
        return linear, angular
