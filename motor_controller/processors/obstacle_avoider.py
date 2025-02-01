#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.node import Node

class ObstacleAvoider:
    """Provides obstacle avoidance behavior for the robot"""
    
    def __init__(self, node: Node,
                 safety_distance: float = 0.4,
                 danger_distance: float = 0.3,
                 max_linear_speed: float = 0.07,
                 max_angular_speed: float = 1.0):
        """
        Initialize obstacle avoider.
        
        Args:
            node: ROS node for logging
            safety_distance: Distance to start avoiding obstacles
            danger_distance: Distance to stop completely
            max_linear_speed: Maximum forward speed
            max_angular_speed: Maximum rotation speed
        """
        self.node = node
        self.safety_distance = safety_distance
        self.danger_distance = danger_distance
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # Add state tracking
        self.is_avoiding = False
        self.avoidance_start_time = None
        self.min_avoidance_time = 2.0
        self.escape_direction = None
        self.last_min_distance = float('inf')

    def get_escape_direction(self, ranges, angles):
        """Determine best escape direction based on obstacle positions"""
        # Split scan into sectors
        front_ranges = ranges[np.where(np.abs(angles) < math.pi/6)[0]]
        left_ranges = ranges[np.where((angles > math.pi/6) & (angles < math.pi/2))[0]]
        right_ranges = ranges[np.where((angles < -math.pi/6) & (angles > -math.pi/2))[0]]
        back_ranges = ranges[np.where(np.abs(angles) > 2*math.pi/3)[0]]
        
        # Get average space in each direction
        spaces = {
            'front': np.mean(front_ranges) if len(front_ranges) > 0 else 0,
            'back': np.mean(back_ranges) if len(back_ranges) > 0 else 0,
            'left': np.mean(left_ranges) if len(left_ranges) > 0 else 0,
            'right': np.mean(right_ranges) if len(right_ranges) > 0 else 0
        }
        
        # Always try to back up if there's any space at all behind
        if spaces['back'] > self.danger_distance:
            return 'back', spaces
            
        # If backing up isn't safe, find direction with most space
        best_direction = max(spaces.items(), key=lambda x: x[1])
        return best_direction[0], spaces

    def process_scan(self, scan: LaserScan, cmd_vel: Twist) -> tuple[Twist, bool]:
        """
        Process laser scan and modify velocity to avoid obstacles.
        Returns (modified velocity command, whether navigation should be paused)
        """
        if not scan:
            return cmd_vel, False

        # Convert scan to numpy array, handling inf values
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = scan.range_max
        
        # Get angles for each range measurement
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # Find closest obstacle in any direction
        min_distance = np.min(ranges)
        min_angle = angles[np.argmin(ranges)]
        
        # Store last min distance for hysteresis
        if not self.is_avoiding:
            self.last_min_distance = min_distance

        # If in danger zone, take action
        if min_distance < self.danger_distance:
            if not self.is_avoiding:
                self.is_avoiding = True
                self.avoidance_start_time = self.node.get_clock().now()
                self.escape_direction, spaces = self.get_escape_direction(ranges, angles)
                self.node.get_logger().warn(
                    f'Obstacle detected at {min_distance:.2f}m, angle: {min_angle:.2f}! '
                    f'Escaping {self.escape_direction} (spaces: {spaces})'
                )
            
            # Create escape command
            modified_cmd = Twist()
            modified_cmd.linear.x = -self.max_linear_speed * 0.7
            return modified_cmd, True
            
        # Check if we should continue avoidance
        elif self.is_avoiding:
            time_avoiding = (self.node.get_clock().now() - self.avoidance_start_time).nanoseconds / 1e9
            
            # Exit avoidance if we've backed up enough and have space
            if time_avoiding > self.min_avoidance_time and min_distance > self.safety_distance:
                self.node.get_logger().info('Exiting avoidance mode - area clear')
                self.is_avoiding = False
                self.escape_direction = None
                return cmd_vel, False
            
            # Continue backing up if still in avoidance
            modified_cmd = Twist()
            modified_cmd.linear.x = -self.max_linear_speed * 0.7
            return modified_cmd, True
            
        # Normal operation
        return cmd_vel, False 