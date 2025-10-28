#!/usr/bin/env python3
"""
SLAM Bot Main Node

Entry point for the ROS2 SLAM robot node. This node orchestrates the autonomous
navigation system by subscribing to lidar and odometry data, processing SLAM 
algorithms, and publishing navigation commands.

Author: Autonomous SLAM Robot ROS2 Project
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from .robot_interface import RobotInterface
from .controller import RobotController


class SlamBotNode(Node):
    """
    Main ROS2 node for SLAM-based autonomous navigation.
    
    This node integrates sensor data (lidar, odometry) with SLAM algorithms
    to enable autonomous exploration and navigation of unknown environments.
    """
    
    def __init__(self):
        super().__init__('slam_bot_node')
        
        # Load configuration parameters
        self.load_parameters()
        
        # Initialize robot interface and controller
        self.robot_interface = RobotInterface(self)
        self.controller = RobotController(
            self,
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel
        )
        
        # Initialize SLAM variables
        self.current_pose = None
        self.current_scan = None
        self.map_data = None
        self.is_initialized = False
        
        # Setup QoS profiles for reliable communication
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.slam_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            self.sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            self.sensor_qos
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            'map',
            self.slam_qos
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'robot_pose',
            10
        )
        
        # Create timer for main control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop_callback
        )
        
        self.get_logger().info('SLAM Bot Node initialized successfully')
        self.get_logger().info(f'Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'Max angular velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'Control frequency: {self.control_frequency} Hz')
    
    def load_parameters(self):
        """
        Load configuration parameters from params.yaml or use defaults.
        """
        # Declare ROS2 parameters with default values
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('lidar_range_min', 0.15)
        self.declare_parameter('lidar_range_max', 12.0)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size', 100)
        
        # Get parameter values
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.lidar_range_min = self.get_parameter('lidar_range_min').value
        self.lidar_range_max = self.get_parameter('lidar_range_max').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_size = self.get_parameter('map_size').value
    
    def lidar_callback(self, msg):
        """
        Process incoming lidar scan data.
        
        Args:
            msg (LaserScan): Incoming laser scan message
        """
        self.current_scan = msg
        
        # Filter out invalid readings
        ranges = np.array(msg.ranges)
        ranges[ranges < self.lidar_range_min] = float('inf')
        ranges[ranges > self.lidar_range_max] = float('inf')
        
        # Update robot interface with latest sensor data
        self.robot_interface.update_lidar_data(ranges, msg.angle_min, msg.angle_increment)
        
        # Log minimal distance for debugging
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            if min_distance < 0.3:  # Obstacle within 30cm
                self.get_logger().warn(f'Close obstacle detected: {min_distance:.2f}m')
    
    def odometry_callback(self, msg):
        """
        Process odometry data from wheel encoders.
        
        Args:
            msg (Odometry): Odometry message containing pose and velocity
        """
        self.current_pose = msg.pose.pose
        
        # Update robot interface with odometry
        self.robot_interface.update_odometry(msg)
        
        # Publish current pose for visualization
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = self.current_pose
        self.pose_pub.publish(pose_stamped)
        
        if not self.is_initialized:
            self.is_initialized = True
            self.get_logger().info('Robot odometry initialized')
    
    def control_loop_callback(self):
        """
        Main control loop executed at regular intervals.
        
        Processes sensor data, updates SLAM, and computes navigation commands.
        """
        if not self.is_initialized or self.current_scan is None:
            # Not ready yet, publish zero velocity
            self.publish_velocity(0.0, 0.0)
            return
        
        try:
            # Get obstacle information from robot interface
            obstacle_data = self.robot_interface.get_obstacle_data()
            
            # Compute navigation command using controller
            linear_vel, angular_vel = self.controller.compute_velocity(
                self.current_pose,
                self.current_scan,
                obstacle_data
            )
            
            # Publish velocity command
            self.publish_velocity(linear_vel, angular_vel)
            
            # Update and publish map (SLAM processing)
            self.update_slam()
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
            self.publish_velocity(0.0, 0.0)
    
    def update_slam(self):
        """
        Update SLAM algorithm and publish occupancy grid map.
        
        This is a placeholder for advanced SLAM algorithms.
        Students can extend this with techniques like:
        - Particle Filter SLAM
        - EKF-SLAM
        - Graph-based SLAM
        - ORB-SLAM for visual SLAM
        """
        # Basic occupancy grid update (simplified)
        # Advanced students: Implement proper SLAM algorithm here
        
        if self.current_scan is None or self.current_pose is None:
            return
        
        # Create occupancy grid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        map_msg.info.origin.position.x = -self.map_size * self.map_resolution / 2.0
        map_msg.info.origin.position.y = -self.map_size * self.map_resolution / 2.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Initialize with unknown values (-1)
        map_msg.data = [-1] * (self.map_size * self.map_size)
        
        # Publish map
        self.map_pub.publish(map_msg)
    
    def publish_velocity(self, linear, angular):
        """
        Publish velocity command to robot.
        
        Args:
            linear (float): Linear velocity in m/s
            angular (float): Angular velocity in rad/s
        """
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)
    
    def shutdown(self):
        """
        Clean shutdown procedure.
        """
        self.get_logger().info('Shutting down SLAM Bot Node')
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    """
    Main entry point for the SLAM bot node.
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        slam_bot = SlamBotNode()
        rclpy.spin(slam_bot)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'slam_bot' in locals():
            slam_bot.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
