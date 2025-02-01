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
        self.min_avoidance_time = 3.0
        self.escape_direction = None

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
        
        # If in danger zone or safety zone, take action
        if min_distance < self.safety_distance:
            if not self.is_avoiding:
                self.is_avoiding = True
                self.avoidance_start_time = self.node.get_clock().now()
                self.escape_direction, spaces = self.get_escape_direction(ranges, angles)
                self.node.get_logger().warn(
                    f'Obstacle detected at {min_distance:.2f}m, angle: {min_angle:.2f}! '
                    f'Escaping {self.escape_direction} (spaces: {spaces})'
                )
            
            # Create escape command with more aggressive backing up
            modified_cmd = Twist()
            
            # Always try to back up first
            modified_cmd.linear.x = -self.max_linear_speed
            
            # Add turning only if really close to obstacle
            if min_distance < self.danger_distance:
                if self.escape_direction == 'left':
                    modified_cmd.angular.z = self.max_angular_speed
                elif self.escape_direction == 'right':
                    modified_cmd.angular.z = -self.max_angular_speed
            
            return modified_cmd, True  # Pause navigation while avoiding
            
        # Check if we should continue avoidance
        if self.is_avoiding:
            time_avoiding = (self.node.get_clock().now() - self.avoidance_start_time).nanoseconds / 1e9
            if time_avoiding < self.min_avoidance_time:
                # Continue escape maneuver
                modified_cmd = Twist()
                if self.escape_direction == 'front':
                    modified_cmd.linear.x = self.max_linear_speed
                elif self.escape_direction == 'back':
                    modified_cmd.linear.x = -self.max_linear_speed
                elif self.escape_direction == 'left':
                    modified_cmd.angular.z = self.max_angular_speed
                else:  # right
                    modified_cmd.angular.z = -self.max_angular_speed
                return modified_cmd, True
            else:
                # End avoidance mode
                self.is_avoiding = False
                self.escape_direction = None
                self.node.get_logger().info('Ending avoidance maneuver')
                return cmd_vel, True  # Request replanning after avoidance

        # If within safety distance, modify velocity
        elif min_distance < self.safety_distance:
            # Calculate avoidance vector
            x_components = ranges * np.cos(angles)
            y_components = ranges * np.sin(angles)
            repulsive_x = np.sum(-1.0 / (x_components + 0.1))
            repulsive_y = np.sum(-1.0 / (y_components + 0.1))
            
            # Calculate avoidance angle
            avoid_angle = math.atan2(repulsive_y, repulsive_x)
            
            # Modify velocity based on proximity
            proximity_factor = (min_distance - self.danger_distance) / (self.safety_distance - self.danger_distance)
            
            modified_cmd = Twist()
            modified_cmd.linear.x = cmd_vel.linear.x * proximity_factor
            modified_cmd.angular.z = cmd_vel.angular.z + (avoid_angle * (1.0 - proximity_factor))
            
            # Clamp values
            modified_cmd.linear.x = max(0.0, min(modified_cmd.linear.x, self.max_linear_speed))
            modified_cmd.angular.z = max(-self.max_angular_speed, min(modified_cmd.angular.z, self.max_angular_speed))
            
            return modified_cmd, False  # Don't pause navigation for minor adjustments
            
        return cmd_vel, False 