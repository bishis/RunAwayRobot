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
        self.reverse_duration = 0.75  # Reduced from 1.5 to 0.75 seconds for faster response
        self.rotation_start_time = None
        self.rotation_duration = 0.5  # Reduced from 1.0 to 0.5 seconds for quicker turns
        self.turn_direction = 'RIGHT'
        
        # Safety parameters - adjusted for faster reaction
        self.safety_distance = 0.5    # Increased from 0.4 to 0.5m to detect obstacles earlier
        self.critical_distance = 0.3  # Increased from 0.25 to 0.3m for earlier stopping
        
        # Expanded scan sectors for better detection
        self.scan_sectors = {
            'front': {'start': 320, 'end': 40, 'min_dist': float('inf')},      # Wider front view
            'front_left': {'start': 20, 'end': 100, 'min_dist': float('inf')}, # Wider left view
            'front_right': {'start': 260, 'end': 340, 'min_dist': float('inf')}, # Wider right view
            'far_left': {'start': 80, 'end': 100, 'min_dist': float('inf')},   # Additional sectors
            'far_right': {'start': 260, 'end': 280, 'min_dist': float('inf')}
        }
        
        # Create timer for faster updates
        self.create_timer(0.05, self.move_robot)  # Increased from 0.1 to 0.05 for faster updates
        
        # Start moving forward
        self.robot_state = RobotState.EXPLORING
        self.motors.set_speeds(1.0, 1.0)
        
    def move_robot(self):
        """Main control loop with priority on obstacle avoidance."""
        # Get all distances
        front_dist = min(self.scan_sectors['front']['min_dist'], 5.0)  # Cap at 5m
        left_dist = min(self.scan_sectors['front_left']['min_dist'], 5.0)
        right_dist = min(self.scan_sectors['front_right']['min_dist'], 5.0)
        far_left = min(self.scan_sectors['far_left']['min_dist'], 5.0)
        far_right = min(self.scan_sectors['far_right']['min_dist'], 5.0)
        
        # Log distances for debugging
        self.get_logger().info(
            f"State: {self.robot_state.name}, "
            f"F: {front_dist:.2f}m, "
            f"L: {left_dist:.2f}m, "
            f"R: {right_dist:.2f}m"
        )
        
        # Handle different states
        if self.robot_state == RobotState.REVERSING:
            if time.time() - self.reverse_start_time >= self.reverse_duration:
                self.start_rotating()
            return
            
        elif self.robot_state == RobotState.ROTATING:
            if time.time() - self.rotation_start_time >= self.rotation_duration:
                # Smart direction choice based on all sensors
                left_space = min(left_dist, far_left)
                right_space = min(right_dist, far_right)
                
                if left_space > right_space + 0.2:  # Prefer left if significantly more space
                    self.turn_direction = 'LEFT'
                elif right_space > left_space + 0.2:  # Prefer right if significantly more space
                    self.turn_direction = 'RIGHT'
                else:  # If space is similar, alternate
                    self.turn_direction = 'LEFT' if self.turn_direction == 'RIGHT' else 'RIGHT'
                
                self.robot_state = RobotState.EXPLORING
                self.motors.set_speeds(1.0, 1.0)
            return
            
        # Enhanced obstacle detection and avoidance
        if front_dist < self.critical_distance or (
            front_dist < self.safety_distance and (left_dist < 0.3 or right_dist < 0.3)
        ):
            # Emergency stop and reverse
            self.get_logger().warn(f"Obstacle too close! F:{front_dist:.2f}m L:{left_dist:.2f}m R:{right_dist:.2f}m")
            self.start_reversing()
            return
            
        elif front_dist < self.safety_distance:
            # Need to turn - choose direction based on available space
            self.get_logger().info("Obstacle ahead, turning")
            # Pre-select turn direction for next rotation
            self.turn_direction = 'LEFT' if left_dist > right_dist else 'RIGHT'
            self.start_rotating()
            return
            
        # If no obstacles, continue forward with slight adjustments
        if self.robot_state == RobotState.EXPLORING:
            # Adjust trajectory based on side distances
            if left_dist < 0.4:  # Too close to left
                self.motors.set_speeds(1.0, 0.8)  # Slight right turn
            elif right_dist < 0.4:  # Too close to right
                self.motors.set_speeds(0.8, 1.0)  # Slight left turn
            else:
                self.motors.set_speeds(1.0, 1.0)  # Full speed ahead

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
        # Update sector distances with improved filtering
        for sector in self.scan_sectors.values():
            if sector['start'] > sector['end']:  # Wrapping around 360
                ranges = msg.ranges[sector['start']:] + msg.ranges[:sector['end']]
            else:
                ranges = msg.ranges[sector['start']:sector['end']]
            
            # Enhanced filtering
            valid_ranges = [r for r in ranges if 0.1 < r < 5.0]  # Ignore very short and long readings
            if valid_ranges:
                # Use average of 3 smallest readings for more stable measurements
                sorted_ranges = sorted(valid_ranges)
                sector['min_dist'] = sum(sorted_ranges[:3]) / min(3, len(sorted_ranges))
            else:
                sector['min_dist'] = float('inf')

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
