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
    EXPLORING = 1    # Moving to frontier
    WALL_FOLLOWING = 2  # Following wall to get around obstacle
    ROTATING = 3     # Turning to new direction

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
        self.wall_follow_direction = 'RIGHT'  # Direction to follow wall
        self.start_point = None  # Starting point when hitting obstacle
        self.min_dist_to_target = float('inf')  # For leave point calculation
        
        # Movement parameters
        self.rotation_threshold = 0.1  # radians
        self.distance_threshold = 0.3  # meters
        self.wall_follow_distance = 0.3  # meters
        
        # Safety parameters
        self.safety_distance = 0.2  # 50cm safety distance
        self.critical_distance = 0.1  # 30cm critical distance
        self.scan_sectors = {
            'front': {'start': 350, 'end': 10, 'min_dist': float('inf')},
            'front_left': {'start': 10, 'end': 30, 'min_dist': float('inf')},
            'front_right': {'start': 330, 'end': 350, 'min_dist': float('inf')}
        }
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Add initial delay before starting
        time.sleep(2.0)
        
        # Create timer for movement control
        self.create_timer(0.1, self.move_robot)
        
        # Start moving forward
        self.start_forward()
        
        # Add frontier exploration
        self.frontier_processor = FrontierProcessor()
        self.map_update_timer = self.create_timer(5.0, self.check_frontiers)
        
        # Add map subscription
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
    def start_forward(self):
        """Start moving forward."""
        self.state = 'FORWARD'
        self.forward_start_time = time.time()
        self.motors.set_speeds(1, 1)  # Move forward at 100% speed
        self.get_logger().info('Moving forward')
        
    def start_turning(self, direction=None):
        """Start turning in specified direction or use default pattern."""
        self.state = 'TURNING'
        self.turn_start_time = time.time()
        
        # If direction is specified, use it; otherwise use alternating pattern
        if direction:
            turn_direction = direction
        else:
            turn_direction = self.turn_direction
            self.turn_direction = 'LEFT' if turn_direction == 'RIGHT' else 'RIGHT'
        
        # Set turn speeds
        if turn_direction == 'RIGHT':
            self.motors.set_speeds(1, -1)
        else:
            self.motors.set_speeds(-1, 1)
            
        self.get_logger().info(f'Turning {turn_direction}')
        
    def check_sector(self, ranges, start_idx, end_idx):
        """Check minimum distance in a sector of the laser scan."""
        if start_idx > end_idx:  # Wrap around case (e.g., 350 to 10)
            sector_ranges = ranges[start_idx:] + ranges[:end_idx]
        else:
            sector_ranges = ranges[start_idx:end_idx]
            
        # Filter out invalid readings (zeros or very small values)
        valid_ranges = [r for r in sector_ranges if r > 0.1]
        return min(valid_ranges) if valid_ranges else float('inf')
        
    def move_robot(self):
        """Main navigation loop using Bug2 algorithm."""
        if self.last_odom is None or self.current_target is None:
            self.motors.stop()
            return

        current_pos = self.last_odom.pose.pose.position
        current_angle = self.get_yaw_from_quaternion(self.last_odom.pose.pose.orientation)
        
        # Calculate distance and angle to target
        dx = self.current_target[0] - current_pos.x
        dy = self.current_target[1] - current_pos.y
        distance_to_target = np.sqrt(dx*dx + dy*dy)
        angle_to_target = np.arctan2(dy, dx)
        
        # Debug info
        self.get_logger().info(
            f"State: {self.robot_state}, "
            f"Distance: {distance_to_target:.2f}m, "
            f"Angle diff: {abs(angle_to_target - current_angle):.2f}rad"
        )

        if distance_to_target < self.distance_threshold:
            # Reached target
            self.get_logger().info("Reached target!")
            self.current_target = None
            self.robot_state = RobotState.EXPLORING
            return

        if self.robot_state == RobotState.EXPLORING:
            # Check if path is clear
            if self.is_path_clear():
                # Move towards target
                self.move_to_target(current_angle, angle_to_target)
            else:
                # Hit obstacle, start wall following
                self.get_logger().info("Obstacle detected, starting wall following")
                self.robot_state = RobotState.WALL_FOLLOWING
                self.start_point = (current_pos.x, current_pos.y)
                self.min_dist_to_target = distance_to_target
                self.wall_follow_direction = 'RIGHT' if self.get_closest_wall_side() == 'RIGHT' else 'LEFT'

        elif self.robot_state == RobotState.WALL_FOLLOWING:
            # Follow wall while checking if we can move to target
            if self.is_path_clear() and distance_to_target < self.min_dist_to_target:
                # Found better path to target
                self.robot_state = RobotState.EXPLORING
                return

            # Update minimum distance to target
            self.min_dist_to_target = min(self.min_dist_to_target, distance_to_target)
            
            # Follow wall
            self.follow_wall()

    def is_path_clear(self):
        """Check if path to target is clear."""
        if not hasattr(self, 'scan_sectors'):
            return True
            
        return self.scan_sectors['front']['min_dist'] > self.wall_follow_distance

    def get_closest_wall_side(self):
        """Determine which side is closer to wall."""
        left_dist = self.scan_sectors['front_left']['min_dist']
        right_dist = self.scan_sectors['front_right']['min_dist']
        return 'LEFT' if left_dist < right_dist else 'RIGHT'

    def follow_wall(self):
        """Follow wall at set distance."""
        if self.wall_follow_direction == 'RIGHT':
            right_dist = self.scan_sectors['front_right']['min_dist']
            if right_dist > self.wall_follow_distance * 1.5:
                # Too far from wall, turn right
                self.motors.set_speeds(0.5, -0.2)
            elif right_dist < self.wall_follow_distance * 0.5:
                # Too close to wall, turn left
                self.motors.set_speeds(-0.2, 0.5)
            else:
                # Good distance, move forward
                self.motors.set_speeds(0.5, 0.5)
        else:
            # Mirror logic for left wall following
            left_dist = self.scan_sectors['front_left']['min_dist']
            if left_dist > self.wall_follow_distance * 1.5:
                self.motors.set_speeds(-0.2, 0.5)
            elif left_dist < self.wall_follow_distance * 0.5:
                self.motors.set_speeds(0.5, -0.2)
            else:
                self.motors.set_speeds(0.5, 0.5)

    def move_to_target(self, current_angle, target_angle):
        """Move towards target position."""
        angle_diff = target_angle - current_angle
        # Normalize angle
        while angle_diff > np.pi: angle_diff -= 2 * np.pi
        while angle_diff < -np.pi: angle_diff += 2 * np.pi
        
        if abs(angle_diff) > self.rotation_threshold:
            # Need to rotate
            turn_speed = 0.3 if angle_diff > 0 else -0.3
            self.motors.set_speeds(-turn_speed, turn_speed)
        else:
            # Correct heading, move forward
            self.motors.set_speeds(0.5, 0.5)

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        # Update sector distances
        for sector in self.scan_sectors.values():
            sector['min_dist'] = self.check_sector(msg.ranges, sector['start'], sector['end'])
        
        # Emergency stop if too close
        front_dist = self.scan_sectors['front']['min_dist']
        if front_dist < self.critical_distance:
            self.motors.stop()
            self.get_logger().warn(f"Emergency stop! Distance: {front_dist:.2f}m")

    def map_callback(self, msg):
        """Process map updates."""
        self.frontier_processor.update_map(msg)
        
    def check_frontiers(self):
        """Check for new frontiers to explore."""
        if self.last_odom is None:
            return
            
        # Update robot position
        self.frontier_processor.update_robot_position(
            self.last_odom.pose.pose.position.x,
            self.last_odom.pose.pose.position.y
        )
        
        # Get new frontier target
        target = self.frontier_processor.get_best_frontier()
        if target:
            self.current_target = target
            self.get_logger().info(f'New exploration target: {target}')
            
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
