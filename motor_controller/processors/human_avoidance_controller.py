#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time

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
        
        # Tracking parameters
        self.min_rotation_speed = 0.05
        self.max_rotation_speed = 0.1
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
            
        # Get parameters from node if available
        if hasattr(node, 'min_rotation_speed'):
            self.min_rotation_speed = node.min_rotation_speed
        if hasattr(node, 'max_angular_speed'):
            self.max_rotation_speed = node.max_angular_speed
        if hasattr(node, 'max_linear_speed'):
            self.max_linear_speed = node.max_linear_speed

        # Add forward motion control
        self.forward_motion_start = 0.0
        self.forward_motion_duration = 0.5  # Forward motion duration in seconds
        self.is_moving_forward = False

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
        
        # Check if we're in forward motion mode
        if self.is_moving_forward:
            if time.time() - self.forward_motion_start < self.forward_motion_duration:
                # Continue forward motion
                cmd.linear.x = 0.1
                self.node.get_logger().info('Continuing forward motion')
                return cmd, True
            else:
                # Stop forward motion
                self.is_moving_forward = False
                self.node.get_logger().info('Forward motion complete - stopping')
                return Twist(), True
        
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
                    needs_escape = True
                    if rear_distance < 0.2:  # 20cm safety threshold
                        self.node.get_logger().warn(
                            f'Wall too close behind ({rear_distance:.2f}m), starting forward motion'
                        )
                        # Start forward motion timer
                        self.is_moving_forward = True
                        self.forward_motion_start = time.time()
                        cmd.linear.x = 0.1
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
            
            # Check rear distance for escape
            if self.latest_scan is not None:
                rear_angle = math.pi  # Only use +180° reading
                angle_tolerance = math.pi/6
                
                start_idx = int((rear_angle - angle_tolerance - self.latest_scan.angle_min) / 
                              self.latest_scan.angle_increment)
                end_idx = int((rear_angle + angle_tolerance - self.latest_scan.angle_min) / 
                             self.latest_scan.angle_increment)
                
                rear_readings = [r for r in self.latest_scan.ranges[start_idx:end_idx]
                               if self.latest_scan.range_min <= r <= self.latest_scan.range_max]
                
                if rear_readings:
                    rear_distance = min(rear_readings)
                    if rear_distance < self.critical_distance:
                        needs_escape = True
                        self.node.get_logger().warn(
                            f'Wall too close behind ({rear_distance:.2f}m), need escape plan'
                        )
                        if rear_distance < 0.2:  # 20cm safety threshold
                            self.node.get_logger().warn(
                                f'Wall too close behind ({rear_distance:.2f}m), moving forward instead'
                            )
                            cmd.linear.x = 0.1  # Small forward motion
        # Debug log the actual commands being sent
        self.node.get_logger().info(
            f'Avoidance command: linear={cmd.linear.x:.2f} m/s, '
            f'angular={cmd.angular.z:.2f} rad/s'
        )
        
        return cmd, needs_escape
        
    def plan_escape(self):
        """Plan escape waypoint furthest from current position"""
        self.node.get_logger().warn('Planning escape')
        
        self.waypoint_generator.force_waypoint_change()
        waypoint = self.waypoint_generator.get_furthest_waypoint()
        
        if waypoint is not None:
            distance = math.sqrt(
                waypoint.pose.position.x**2 + 
                waypoint.pose.position.y**2
            )
            if distance > 1.0:  # Minimum escape distance
                waypoint.header.frame_id = 'map'
                waypoint.header.stamp.nanosec = 1  # Special marker for escape
                
                # Create red visualization for escape waypoint
                markers = self.waypoint_generator.create_visualization_markers(waypoint, is_escape=True)
                self.node.marker_pub.publish(markers)
                
                self.node.get_logger().warn(
                    f'ESCAPE PLAN: Moving to ({waypoint.pose.position.x:.2f}, '
                    f'{waypoint.pose.position.y:.2f}), ignoring humans until reached'
                )
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