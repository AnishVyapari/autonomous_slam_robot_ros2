#!/usr/bin/env python3
"""
Robot Interface Module

This module abstracts low-level hardware interactions for the differential-drive robot.
It handles wheel encoders, IMU fusion (if available), and lidar preprocessing to provide
cleaned and fused state estimates to higher-level controllers.

Author: Autonomous SLAM Robot ROS2 Project
License: MIT
"""

from typing import Tuple, Optional
import numpy as np
from nav_msgs.msg import Odometry


class RobotInterface:
    """
    Interface between ROS topics and robot state estimation.

    For real robots, this class should contain hardware drivers or bridges to
    microcontroller firmware. In simulation, it processes ROS messages to
    estimate pose and obstacle information.
    """

    def __init__(self, node, wheel_base: float = 0.16, wheel_radius: float = 0.033):
        """
        Args:
            node (rclpy.node.Node): Parent ROS2 node for logging and time
            wheel_base (float): Distance between wheels (meters)
            wheel_radius (float): Radius of the wheels (meters)
        """
        self.node = node
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

        # State variables
        self.last_odom: Optional[Odometry] = None
        self.lidar_ranges: Optional[np.ndarray] = None
        self.lidar_angle_min: float = 0.0
        self.lidar_angle_increment: float = 0.0

    # ---------------------- Sensor Update Methods ----------------------
    def update_odometry(self, odom_msg: Odometry) -> None:
        """Store latest odometry message.
        Args:
            odom_msg: Incoming Odometry message
        """
        self.last_odom = odom_msg

    def update_lidar_data(self, ranges: np.ndarray, angle_min: float, angle_increment: float) -> None:
        """Store processed lidar readings and metadata.
        Args:
            ranges: 1D numpy array of lidar distances
            angle_min: starting angle of the scan
            angle_increment: angle between two consecutive rays
        """
        self.lidar_ranges = ranges
        self.lidar_angle_min = angle_min
        self.lidar_angle_increment = angle_increment

    # ------------------------- Query Methods ---------------------------
    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return robot pose (x, y, yaw) if available.
        Returns:
            (x, y, yaw) tuple or None if odometry not available
        """
        if self.last_odom is None:
            return None
        pose = self.last_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        # Convert quaternion to yaw
        q = pose.orientation
        yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return (x, y, yaw)

    def get_obstacle_data(self) -> dict:
        """Compute simple obstacle descriptors from the lidar scan.
        Returns:
            dict with keys: 'front', 'left', 'right', 'min_distance'
        """
        data = {'front': np.inf, 'left': np.inf, 'right': np.inf, 'min_distance': np.inf}
        if self.lidar_ranges is None:
            return data

        ranges = self.lidar_ranges
        n = len(ranges)
        if n == 0:
            return data

        # Sector indices (assuming 0 at angle_min and increasing)
        # Front: center 30 degrees, Left: +60 to +120, Right: -120 to -60
        fov_deg = 30
        sector = int(np.deg2rad(fov_deg) / max(self.lidar_angle_increment, 1e-6))
        center = n // 2
        left_start = int(center + (np.deg2rad(60) / max(self.lidar_angle_increment, 1e-6)))
        left_end = int(center + (np.deg2rad(120) / max(self.lidar_angle_increment, 1e-6)))
        right_start = int(center - (np.deg2rad(120) / max(self.lidar_angle_increment, 1e-6)))
        right_end = int(center - (np.deg2rad(60) / max(self.lidar_angle_increment, 1e-6)))

        front_slice = ranges[max(0, center - sector):min(n, center + sector)]
        left_slice = ranges[max(0, left_start):min(n, left_end)]
        right_slice = ranges[max(0, right_start):min(n, right_end)]

        def safe_min(arr):
            arr = arr[np.isfinite(arr)]
            return float(np.min(arr)) if arr.size else float('inf')

        data['front'] = safe_min(front_slice)
        data['left'] = safe_min(left_slice)
        data['right'] = safe_min(right_slice)
        data['min_distance'] = safe_min(ranges)
        return data

    # ---------------------- Utility Methods ----------------------------
    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw angle (in radians)."""
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))
