#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time
from .human_escape import HumanEscape

class HumanAvoidanceController:
    def __init__(self, node, waypoint_generator):
        """
        Initialize the human avoidance controller
        
        Args:
            node: The ROS node to use for logging and parameters
            waypoint_generator: WaypointGenerator instance for exploration
        """
        self.node = node
        self.waypoint_generator = waypoint_generator
        # Add escape planner
        self.escape_planner = HumanEscape(node)
        
        # Distance thresholds
        self.min_safe_distance = 1.5  # Start backing up at 1m
        self.critical_distance = 0.35   # Request escape at 0.3m
        self.max_backup_speed = 0.15   # Increase backup speed
        
        # Tracking parameters
        self.min_rotation_speed = 0.05
        self.max_rotation_speed = 0.06
        self.turn_p_gain = 0.6
        
        # Frame zones - simpler tracking
        self.center_zone = 0.25  # Consider centered within ±25% of frame center
        
        # Tracking state
        self.last_image_x = None
        self.is_turning = False
        
        # Get latest scan data from node
        self.latest_scan = None
        if hasattr(node, 'latest_scan'):
            self.latest_scan = node.latest_scan
            

        # Add forward motion control
        self.forward_motion_start = 0.0
        self.forward_motion_duration = 0.5  # Forward motion duration in seconds

    def calculate_turn_command(self, image_x: float) -> float:
        """Calculate turn speed to face human."""
        # Calculate normalized error from image center (-1 to 1)
        image_center = 320
        normalized_error = (image_x - image_center) / image_center
        
        # Check if human is in center zone
        if abs(normalized_error) <= self.center_zone:
            self.node.get_logger().info('Human centered - no turn needed')
            self.is_turning = False
            return 0.0
            
        # Calculate turn direction and speed
        turn_direction = 1.0 if normalized_error > 0 else -1.0
        
        # Use constant turn speed based on direction
        turn_speed = self.max_rotation_speed * turn_direction
        
        self.node.get_logger().info(
            f'Turning to face human: error={normalized_error:.2f}, speed={turn_speed:.2f}'
        )
        self.is_turning = True
        return turn_speed

    def get_avoidance_command(self, human_distance, human_angle, image_x=None):
        cmd = Twist()
        needs_escape = False
        
        
        # Check rear 140° arc of robot using LIDAR
        if self.latest_scan is not None:
            # Define rear arc from 110° to 250° for 140° total coverage
            rear_start_angle = 100 * math.pi/180  # 110 degrees
            rear_end_angle = 240 * math.pi/180    # 250 degrees
            
            # Convert angles to LIDAR indices
            start_idx = int((rear_start_angle - self.latest_scan.angle_min) / 
                          self.latest_scan.angle_increment)
            end_idx = int((rear_end_angle - self.latest_scan.angle_min) / 
                         self.latest_scan.angle_increment)
            
            # Ensure indices are within bounds
            start_idx = max(0, min(start_idx, len(self.latest_scan.ranges)-1))
            end_idx = max(0, min(end_idx, len(self.latest_scan.ranges)-1))
            
            # Get valid readings in rear arc
            rear_readings = []
            rear_angles = []
            
            for i in range(start_idx, end_idx + 1):
                r = self.latest_scan.ranges[i]
                if self.latest_scan.range_min <= r <= self.latest_scan.range_max:
                    rear_readings.append(r)
                    angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
                    rear_angles.append(angle)
            
            if rear_readings:
                # Find closest point and its angle
                min_idx = rear_readings.index(min(rear_readings))
                rear_distance = rear_readings[min_idx]
                closest_angle = rear_angles[min_idx]
                
                self.node.get_logger().info(
                    f'Rear distance: {rear_distance:.2f}m at angle: {math.degrees(closest_angle):.1f}°'
                )
                
                # Check if we need to escape
                if rear_distance < self.critical_distance:
                    needs_escape = True
                    return cmd, needs_escape
        
        # Handle turning to face human
        if image_x is not None:
            # Calculate turn command
            cmd.angular.z = self.calculate_turn_command(image_x)
            
            if self.is_turning:
                # If turning, only turn (no backup)
                self.node.get_logger().info('Turning to face human')
                return cmd, False
            else:
                # Human is centered - stop turning
                stop_cmd = Twist()
                
                # Only allow backup when human is centered
                if human_distance < self.min_safe_distance:
                    stop_cmd.linear.x = -self.max_backup_speed
                    self.node.get_logger().info('Human centered - backing up')
                
                return stop_cmd, False
                
        else:  # No human detected
            # Stop all motion
            stop_cmd = Twist()
            self.is_turning = False
            self.last_image_x = None
            return stop_cmd, False

        
    def plan_escape(self):
        """Plan escape route when human is too close"""
        self.node.get_logger().info('Planning escape route...')
        
        # Ensure we have current map data
        if not self.escape_planner.current_map:
            self.node.get_logger().error('No map data available for escape planning!')
            return None
        
        # Use dedicated escape planner
        escape_point = self.escape_planner.get_furthest_waypoint()
        
        if escape_point is not None:
            self.node.get_logger().info(
                f'Found escape point at ({escape_point.pose.position.x:.2f}, '
                f'{escape_point.pose.position.y:.2f})'
            )
            return escape_point
        else:
            self.node.get_logger().error('Failed to find escape point!')
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