#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Quaternion
import tf2_ros
import math
import time
import numpy as np
from enum import Enum

from .controllers.motor_controller import MotorController
from .controllers.navigation_controller import NavigationController
from .processors.lidar_processor import LidarProcessor

class RobotState(Enum):
    EXPLORING = 1    # Moving to frontier
    ROTATING = 2     # Turning to new direction
    REVERSING = 3    # Backing away from obstacle
    SCANNING = 4     # Scanning area for mapping

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize controllers
        self.motors = MotorController(left_pin=18, right_pin=12)
        self.lidar_processor = LidarProcessor()
        
        # Navigation parameters
        self.current_pose = None
        self.last_scan = None
        self.target_yaw = 0  # Current target orientation (0 = forward)
        self.leg_length = 2.0  # Length of each straight segment in meters
        self.current_leg = 0  # Track current segment
        self.start_position = None
        self.is_turning = False
        
        # Setup ROS subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        
        # Create control timer
        self.create_timer(0.05, self.control_loop)
    
    def control_loop(self):
        """Main control loop for square wave pattern movement."""
        if self.current_pose is None or self.last_scan is None:
            return

        # Get current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Initialize start position if not set
        if self.start_position is None:
            self.start_position = (current_x, current_y)
            
        # Process LIDAR data
        sector_data = self.lidar_processor.process_scan(self.last_scan.ranges)
        if not sector_data:
            self.motors.stop()
            return

        # Check if path is clear
        path_clear = self.lidar_processor.get_navigation_command(sector_data)
        if not path_clear:
            self.motors.stop()
            return

        # Calculate distance traveled in current leg
        if self.current_leg % 2 == 0:  # Even legs (forward)
            distance = abs(current_y - self.start_position[1])
        else:  # Odd legs (sideways)
            distance = abs(current_x - self.start_position[0])

        # Check if we need to turn
        if distance >= self.leg_length:
            if not self.is_turning:
                self.is_turning = True
                self.target_yaw = (self.current_leg + 1) * 90  # Turn 90 degrees
                if self.target_yaw >= 360:
                    self.target_yaw -= 360
            
            # Handle turning
            angle_diff = self.target_yaw - math.degrees(current_yaw)
            if abs(angle_diff) > 5:  # Allow 5 degrees of error
                self.motors.turn_left() if angle_diff > 0 else self.motors.turn_right()
            else:
                self.is_turning = False
                self.current_leg += 1
                self.start_position = (current_x, current_y)
        else:
            # Move forward if path is clear
            self.motors.forward(speed=0.8)

        # Log current state
        self.get_logger().info(
            f"Leg: {self.current_leg}, Distance: {distance:.2f}m, "
            f"Yaw: {math.degrees(current_yaw):.1f}°, Target: {self.target_yaw}°"
        )

    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    robot = MobileRobotController()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Stopping robot")
        robot.motors.stop()
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
