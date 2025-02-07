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
        
        # Parameters
        self.min_safe_distance = 1.5  # Meters
        self.max_backup_speed = 0.15  # m/s
        self.min_escape_distance = 2.0  # Minimum distance for escape waypoint
        
        # Add tracking parameters
        self.frame_width = 640  # Default camera width
        self.center_threshold = 0.3  # Keep human within 30% of center
        self.max_turn_speed = 0.8
        self.min_turn_speed = 0.3
        self.turn_p_gain = 0.8  # P controller gain for turning
        
        # Get latest scan data from node
        self.latest_scan = None
        if hasattr(node, 'latest_scan'):
            self.latest_scan = node.latest_scan
            
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
        
        # Calculate turning command to keep human in frame
        if image_x is not None:
            # Normalize image position to -1 to 1
            normalized_x = (2 * image_x / self.frame_width) - 1
            
            # Only turn if human is too far from center
            if abs(normalized_x) > self.center_threshold:
                # P controller for turning
                turn_speed = -normalized_x * self.turn_p_gain
                
                # Ensure minimum turn speed when needed
                if abs(turn_speed) < self.min_turn_speed and turn_speed != 0:
                    turn_speed = math.copysign(self.min_turn_speed, turn_speed)
                
                # Limit maximum turn speed
                turn_speed = max(min(turn_speed, self.max_turn_speed), -self.max_turn_speed)
                cmd.angular.z = turn_speed
                
                self.node.get_logger().info(
                    f'Tracking human: pos={normalized_x:.2f}, turn={turn_speed:.2f}'
                )
        
        # Check if we're too close to the human
        if human_distance < self.min_safe_distance:
            self.node.get_logger().info(f'Human too close ({human_distance:.2f}m), backing up')
            
            # Calculate backup speed based on how close the human is
            backup_scale = 1.0 - (human_distance / self.min_safe_distance)
            backup_speed = -self.max_backup_speed * backup_scale
            
            # Check if we can back up
            if self.can_move_backward():
                cmd.linear.x = backup_speed
                self.node.get_logger().info(f'Backing up at {backup_speed:.2f} m/s')
                
                # If we're not tracking in image, use angle-based turning
                if image_x is None:
                    # Turn away from human while backing up
                    turn_direction = math.copysign(1.0, -human_angle)
                    cmd.angular.z = turn_direction * 0.5
            else:
                self.node.get_logger().warn('Cannot back up further, need escape plan')
                needs_escape = True
        
        return cmd, needs_escape
        
    def can_move_backward(self):
        """Check if there's space to move backward"""
        if self.latest_scan is None:
            return False
            
        # Check rear LIDAR readings (assume LIDAR 0 is front, ±π is rear)
        rear_angles = [-math.pi, math.pi]  # Check both sides of rear
        angle_tolerance = math.pi/6  # 30 degree cone behind robot
        
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
        
        # Need at least 0.5m to back up
        return min_rear_distance > 0.5
        
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