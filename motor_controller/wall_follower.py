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
        
        # Add LIDAR subscription back
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Add map subscription
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Add variables for scan data
        self.latest_scan = None
        self.scan_time = None
        self.scan_timeout = 0.5  # seconds

    def move_robot(self):
        """Main control loop with priority on obstacle avoidance."""
        # Check if we have recent scan data
        if self.latest_scan is None or (time.time() - self.scan_time) > self.scan_timeout:
            self.get_logger().warn("No recent LIDAR data - stopping!")
            self.motors.stop()
            return

        # Get all distances
        front_dist = min(self.scan_sectors['front']['min_dist'], 5.0)
        left_dist = min(self.scan_sectors['front_left']['min_dist'], 5.0)
        right_dist = min(self.scan_sectors['front_right']['min_dist'], 5.0)
        far_left = min(self.scan_sectors['far_left']['min_dist'], 5.0)
        far_right = min(self.scan_sectors['far_right']['min_dist'], 5.0)

        # Double check front distance with raw scan data
        front_ranges = self.latest_scan.ranges[350:10]  # Direct front view
        valid_ranges = [r for r in front_ranges if 0.1 < r < 5.0]
        if valid_ranges:
            direct_front = min(valid_ranges)
            front_dist = min(front_dist, direct_front)  # Use the smaller value

        # Log distances and state
        self.get_logger().info(
            f"State: {self.robot_state.name}, "
            f"F: {front_dist:.2f}m, "
            f"L: {left_dist:.2f}m, "
            f"R: {right_dist:.2f}m, "
            f"Direct Front: {direct_front:.2f}m"
        )

        # More aggressive obstacle detection
        if front_dist < self.critical_distance or direct_front < self.critical_distance:
            self.get_logger().warn(f"CRITICAL - Obstacle very close! Front: {front_dist:.2f}m")
            self.motors.stop()
            self.start_reversing()
            return
            
        elif front_dist < self.safety_distance or direct_front < self.safety_distance:
            self.get_logger().warn(f"WARNING - Obstacle ahead! Front: {front_dist:.2f}m")
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
        self.latest_scan = msg
        self.scan_time = time.time()
        
        # Update sector distances with improved filtering
        for sector in self.scan_sectors.values():
            if sector['start'] > sector['end']:  # Wrapping around 360
                ranges = msg.ranges[sector['start']:] + msg.ranges[:sector['end']]
            else:
                ranges = msg.ranges[sector['start']:sector['end']]
            
            # Enhanced filtering - use minimum of valid readings
            valid_ranges = [r for r in ranges if 0.1 < r < 5.0]
            if valid_ranges:
                # Take minimum of valid readings for safety
                sector['min_dist'] = min(valid_ranges)
            else:
                sector['min_dist'] = float('inf')

        # Emergency check - if any reading in front is too close, stop immediately
        front_readings = msg.ranges[350:] + msg.ranges[:10]
        critical_readings = [r for r in front_readings if 0.1 < r < self.critical_distance]
        if critical_readings:
            self.get_logger().error(f"Emergency - Obstacle detected at {min(critical_readings):.2f}m!")
            self.motors.stop()
            self.start_reversing()

    def map_callback(self, msg):
        """Process map updates."""
        # Store map data for obstacle checking
        self.map_data = msg.data
        self.map_info = msg.info

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
