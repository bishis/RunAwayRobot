#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np
from enum import Enum

from .controllers.motor_controller import MotorController
from .processors.lidar_processor import LidarProcessor
from .processors.frontier_processor import FrontierProcessor

class RobotState(Enum):
    EXPLORING = 1    # Moving forward or to frontier
    AVOIDING = 2     # Avoiding obstacle
    ROTATING = 3     # Turning to new direction
    REVERSING = 4    # Backing away from obstacle

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize components
        self.motors = MotorController(
            left_pin=18,
            right_pin=12
        )
        self.lidar = LidarProcessor()
        
        # Navigation state
        self.robot_state = RobotState.EXPLORING
        self.current_target = None
        self.reverse_start_time = None
        self.reverse_duration = 1.5  # seconds
        self.rotation_start_time = None
        self.rotation_duration = 1.0  # seconds
        self.turn_direction = 'RIGHT'  # Alternates between LEFT and RIGHT
        
        # Safety parameters
        self.safety_distance = 0.4    # 40cm safety distance
        self.critical_distance = 0.25  # 25cm critical distance
        self.scan_sectors = {
            'front': {'start': 330, 'end': 30, 'min_dist': float('inf')},
            'front_left': {'start': 30, 'end': 90, 'min_dist': float('inf')},
            'front_right': {'start': 270, 'end': 330, 'min_dist': float('inf')}
        }
        
        # Setup ROS components
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Create timer for movement control
        self.create_timer(0.1, self.move_robot)
        
        # Start moving forward
        self.robot_state = RobotState.EXPLORING
        self.motors.set_speeds(1.0, 1.0)
        
    def move_robot(self):
        """Main control loop with priority on obstacle avoidance."""
        # Always check obstacles first
        front_dist = self.scan_sectors['front']['min_dist']
        left_dist = self.scan_sectors['front_left']['min_dist']
        right_dist = self.scan_sectors['front_right']['min_dist']
        
        # Log distances for debugging
        self.get_logger().info(
            f"State: {self.robot_state.name}, "
            f"Distances - Front: {front_dist:.2f}m, "
            f"Left: {left_dist:.2f}m, "
            f"Right: {right_dist:.2f}m"
        )
        
        # Handle different states
        if self.robot_state == RobotState.REVERSING:
            if time.time() - self.reverse_start_time >= self.reverse_duration:
                self.start_rotating()
            return
            
        elif self.robot_state == RobotState.ROTATING:
            if time.time() - self.rotation_start_time >= self.rotation_duration:
                # Choose direction with most space
                if left_dist > right_dist:
                    self.turn_direction = 'LEFT'
                else:
                    self.turn_direction = 'RIGHT'
                self.robot_state = RobotState.EXPLORING
                self.motors.set_speeds(1.0, 1.0)  # Move forward
            return
            
        # Check for obstacles and take appropriate action
        if front_dist < self.critical_distance:
            # Emergency stop and reverse
            self.get_logger().warn(f"Obstacle too close! Distance: {front_dist:.2f}m")
            self.start_reversing()
            return
            
        elif front_dist < self.safety_distance:
            # Need to turn
            self.get_logger().info("Obstacle ahead, turning")
            self.start_rotating()
            return
            
        # If no obstacles, continue forward
        if self.robot_state == RobotState.EXPLORING:
            self.motors.set_speeds(1.0, 1.0)

    def start_reversing(self):
        """Start reversing away from obstacle."""
        self.get_logger().info("Starting to reverse")
        self.robot_state = RobotState.REVERSING
        self.reverse_start_time = time.time()
        self.motors.set_speeds(-1.0, -1.0)
        
    def start_rotating(self):
        """Start rotating to avoid obstacle."""
        self.get_logger().info(f"Starting to rotate {self.turn_direction}")
        self.robot_state = RobotState.ROTATING
        self.rotation_start_time = time.time()
        
        if self.turn_direction == 'LEFT':
            self.motors.set_speeds(-1.0, 1.0)
            self.turn_direction = 'RIGHT'  # Alternate direction for next time
        else:
            self.motors.set_speeds(1.0, -1.0)
            self.turn_direction = 'LEFT'  # Alternate direction for next time

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        # Update sector distances with filtering for noise
        for sector in self.scan_sectors.values():
            if sector['start'] > sector['end']:  # Wrapping around 360
                ranges = msg.ranges[sector['start']:] + msg.ranges[:sector['end']]
            else:
                ranges = msg.ranges[sector['start']:sector['end']]
            
            # Filter out invalid readings and take minimum
            valid_ranges = [r for r in ranges if r > 0.1 and r < 5.0]
            sector['min_dist'] = min(valid_ranges) if valid_ranges else float('inf')

    def map_callback(self, msg):
        """Process map updates."""
        pass  # We're not using the map for basic exploration

def main(args=None):
    rclpy.init(args=args)
    robot = MobileRobotController()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Stopping motors")
        robot.motors.stop()
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
