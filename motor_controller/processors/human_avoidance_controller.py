#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class HumanAvoidanceController:
    def __init__(self, node, waypoint_generator):
        """
        Initialize the human avoidance controller
        
        Args:
            node: The ROS node to use for logging and parameters
            waypoint_generator: WaypointGenerator instance for escape planning
        """
        self.node = node
        self.waypoint_generator = waypoint_generator
        
        # Distance thresholds
        self.min_safe_distance = 1.0  # Start backing up at 1m
        self.critical_distance = 0.3   # Request escape at 0.3m
        self.max_backup_speed = 0.15   # m/s
        
        # Turning parameters
        self.min_rotation_speed = 0.3
        self.max_rotation_speed = 0.8
        self.turn_p_gain = 1.2  # Increased for more responsive turning
        
        # Get latest scan data from node
        self.latest_scan = None
        if hasattr(node, 'latest_scan'):
            self.latest_scan = node.latest_scan
            
        # Add tracking parameters
        self.frame_width = 640  # Default camera width
        self.center_threshold = 0.5  # Keep human within 50% of center
        self.max_turn_speed = 0.8
        self.min_turn_speed = 0.3
        self.max_linear_speed = 0.07   # Maximum linear speed
        
        # Get parameters from node if available
        if hasattr(node, 'min_rotation_speed'):
            self.min_rotation_speed = node.min_rotation_speed
        if hasattr(node, 'max_angular_speed'):
            self.max_rotation_speed = node.max_angular_speed
        if hasattr(node, 'max_linear_speed'):
            self.max_linear_speed = node.max_linear_speed
            
    def get_avoidance_command(self, human_distance, human_angle, image_x=None):
        """
        Calculate avoidance command based on human position
        
        Args:
            human_distance: Distance to human in meters
            human_angle: Angle to human in radians
            image_x: Human's x position in image (0 to frame_width), if available
        
        Returns:
            Tuple(Twist, bool): (velocity command, needs_escape_plan)
        """
        cmd = Twist()
        needs_escape = False
        
        # Log the incoming distance
        self.node.get_logger().info(f'Human distance: {human_distance:.2f}m')
        
        # Always try to face the human when detected
        if image_x is not None:
            # Calculate normalized error from image center (-1 to 1)
            image_center = 320  # Assuming 640x480 image
            normalized_error = (image_x - image_center) / image_center
            
            # Calculate turn speed to face human
            turn_speed = normalized_error * self.turn_p_gain
            
            # Apply minimum rotation speed if turning
            if abs(turn_speed) > 0.0:
                if abs(turn_speed) < self.min_rotation_speed:
                    turn_speed = math.copysign(self.min_rotation_speed, turn_speed)
                    cmd.linear.x = 0.0  # Stop while turning slowly
            
            # Limit maximum rotation speed
            cmd.angular.z = max(min(turn_speed, self.max_rotation_speed), 
                              -self.max_rotation_speed)
            
            self.node.get_logger().info(
                f'Facing human: error={normalized_error:.2f}, turn={cmd.angular.z:.2f}'
            )
        
        # Check distances and respond accordingly
        if human_distance < self.min_safe_distance:
            # Start backing up when closer than safe distance
            self.node.get_logger().warn(
                f'Human too close ({human_distance:.2f}m < {self.min_safe_distance:.2f}m), backing up'
            )
            
            # Calculate backup speed based on how close the human is
            backup_scale = 1.0 - (human_distance / self.min_safe_distance)
            backup_speed = -self.max_backup_speed * backup_scale
            
            # Check if we can back up
            if self.can_move_backward():
                cmd.linear.x = backup_speed
                self.node.get_logger().warn(
                    f'Backing up at {backup_speed:.2f} m/s (scale={backup_scale:.2f})'
                )
                
                # Keep facing human while backing up
                if image_x is None and human_angle is not None:
                    turn_speed = math.copysign(0.5, -human_angle)
                    cmd.angular.z = max(min(turn_speed, self.max_rotation_speed), 
                                      -self.max_rotation_speed)
            else:
                # If we can't back up and human is very close, request escape
                if human_distance < self.critical_distance:
                    self.node.get_logger().error(
                        f'Cannot back up and human too close ({human_distance:.2f}m), need escape plan'
                    )
                    needs_escape = True
        
        # Ensure we're not exceeding max speeds
        cmd.linear.x = max(min(cmd.linear.x, self.max_linear_speed), -self.max_linear_speed)
        cmd.angular.z = max(min(cmd.angular.z, self.max_rotation_speed), -self.max_rotation_speed)
        
        return cmd, needs_escape
        
    def can_move_backward(self):
        """Check if there's space to move backward"""
        if self.latest_scan is None:
            return False
            
        # Check rear LIDAR readings (assume LIDAR 0 is front, ±π is rear)
        rear_angles = [-math.pi, math.pi]  # Check both sides of rear
        angle_tolerance = math.pi/6  # 30 degree cone behind robot
        min_backup_distance = 0.3  # Minimum space needed to back up
        
        min_rear_distance = float('inf')
        
        for angle in rear_angles:
            start_idx = int((angle - angle_tolerance - self.latest_scan.angle_min) / 
                          self.latest_scan.angle_increment)
            end_idx = int((angle + angle_tolerance - self.latest_scan.angle_min) / 
                         self.latest_scan.angle_increment)
            
            # Get valid readings in rear arc
            rear_readings = [r for r in self.latest_scan.ranges[start_idx:end_idx]
                           if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
            
            if rear_readings:
                min_rear_distance = min(min_rear_distance, min(rear_readings))
        
        return min_rear_distance > min_backup_distance
        
    def plan_escape(self):
        """Plan escape waypoint furthest from current position"""
        # Force waypoint generator to pick furthest point
        self.waypoint_generator.force_waypoint_change()
        waypoint = self.waypoint_generator.get_furthest_waypoint()
        
        if waypoint is not None and waypoint.position.x**2 + waypoint.position.y**2 > self.min_escape_distance**2:
            self.node.get_logger().info('Found escape waypoint')
            return waypoint
            
        self.node.get_logger().warn('Could not find suitable escape waypoint')
        return None 