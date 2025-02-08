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
        self.min_safe_distance = 1.5  # Start backing up at 1m
        self.critical_distance = 0.3   # Request escape at 0.3m
        self.max_backup_speed = 0.15   # Increase backup speed
        
        # Turning parameters
        self.min_rotation_speed = 0.3
        self.max_rotation_speed = 0.8
        self.turn_p_gain = 1.2
        
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
            
        # Add timer to continuously monitor rear distance
        self.node.create_timer(0.2, self.monitor_rear_distance)  # 5Hz updates
        
    def get_avoidance_command(self, human_distance, human_angle, image_x=None):
        cmd = Twist()
        needs_escape = False
        
        # Check rear distance first using monitor method
        if self.latest_scan is not None:
            rear_angle = math.pi  # Only use +180° reading
            angle_tolerance = math.pi/6
            
            start_idx = int((rear_angle - angle_tolerance - self.latest_scan.angle_min) / 
                          self.latest_scan.angle_increment)
            end_idx = int((rear_angle + angle_tolerance - self.latest_scan.angle_min) / 
                         self.latest_scan.angle_increment)
            
            # Get valid readings in rear arc
            rear_readings = [r for r in self.latest_scan.ranges[start_idx:end_idx]
                           if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
            
            if rear_readings:
                rear_distance = min(rear_readings)
                self.node.get_logger().info(f'Rear wall distance: {rear_distance:.2f}m')
                
                # Check if we need to escape
                if rear_distance < self.critical_distance:
                    self.node.get_logger().warn(
                        f'Wall too close behind ({rear_distance:.2f}m), need escape plan'
                    )
                    needs_escape = True
                    return cmd, needs_escape

        # Handle turning FIRST - prioritize facing the human
        if image_x is not None:
            # Calculate normalized error from image center (-1 to 1)
            image_center = 320  # Assuming 640x480 image
            normalized_error = (image_x - image_center) / image_center
            
            # Calculate turn speed to face human
            turn_speed = normalized_error * self.turn_p_gain
            
            # Apply minimum rotation speed if turning
            if abs(normalized_error) > 0.1:  # Only turn if error is significant
                if abs(turn_speed) < self.min_rotation_speed:
                    turn_speed = math.copysign(self.min_rotation_speed, turn_speed)
                cmd.angular.z = max(min(turn_speed, self.max_rotation_speed), 
                                  -self.max_rotation_speed)
                
                # Don't back up while making large turns
                if abs(normalized_error) > 0.3:
                    self.node.get_logger().info(
                        f'Turning to face human first: error={normalized_error:.2f}'
                    )
                    return cmd, False
        
        # Only start backing up if we're roughly facing the human
        if human_distance < self.min_safe_distance:
            self.node.get_logger().info(
                f'Human too close ({human_distance:.2f}m), backing up'
            )
            
            # Calculate backup speed
            backup_scale = (self.min_safe_distance - human_distance) / self.min_safe_distance
            backup_speed = -self.max_backup_speed * max(0.8, backup_scale)
            
            # Set backup speed
            cmd.linear.x = backup_speed
            self.node.get_logger().info(f'Setting backup speed to {backup_speed:.2f} m/s')
            
            # Check if we can back up
            if not self.can_move_backward():
                if human_distance < self.critical_distance:
                    needs_escape = True
                    self.node.get_logger().warn('Cannot back up, need escape plan')
        
        # Debug log the actual commands being sent
        self.node.get_logger().info(
            f'Avoidance command: linear={cmd.linear.x:.2f} m/s, '
            f'angular={cmd.angular.z:.2f} rad/s'
        )
        
        return cmd, needs_escape
        
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

    def monitor_rear_distance(self):
        """Continuously monitor and log distance behind robot"""
        if self.latest_scan is None:
            self.node.get_logger().warn('No scan data available for rear monitoring')
            return
            
        # Check rear LIDAR readings
        rear_angles = [-math.pi, math.pi]  # Check both sides of rear
        angle_tolerance = math.pi/6  # 30 degree cone behind robot
        
        self.node.get_logger().info('=== Rear Distance Monitor ===')
        
        min_rear_distance = float('inf')
        distances = []
        
        for angle in rear_angles:
            start_idx = int((angle - angle_tolerance - self.latest_scan.angle_min) / 
                          self.latest_scan.angle_increment)
            end_idx = int((angle + angle_tolerance - self.latest_scan.angle_min) / 
                         self.latest_scan.angle_increment)
            
            # Get all readings (including invalid ones)
            all_readings = self.latest_scan.ranges[start_idx:end_idx]
            if len(all_readings) > 0:
                self.node.get_logger().info(
                    f'Raw readings at {math.degrees(angle):.1f}°: '
                    f'min={min(all_readings):.2f}m, max={max(all_readings):.2f}m'
                )
            
            # Get valid readings in rear arc
            rear_readings = [r for r in all_readings
                           if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
            
            if rear_readings:
                arc_min_distance = min(rear_readings)
                distances.append(arc_min_distance)
                min_rear_distance = min(min_rear_distance, arc_min_distance)
                self.node.get_logger().info(
                    f'Valid distance at {math.degrees(angle):.1f}°: {arc_min_distance:.2f}m'
                )
            else:
                self.node.get_logger().warn(
                    f'No valid readings at {math.degrees(angle):.1f}°'
                )
        
        if distances:
            self.node.get_logger().info(
                f'Minimum wall distance behind robot: {min_rear_distance:.2f}m'
            )
        else:
            self.node.get_logger().warn('No valid distances behind robot!')
        
        self.node.get_logger().info('===========================') 