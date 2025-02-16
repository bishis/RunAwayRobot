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
        
        # Tracking parameters - reduce maximum speeds
        self.min_rotation_speed = 0.05    # Reduced from 0.3
        self.max_rotation_speed = 0.1    # Reduced from 0.75
        self.turn_p_gain = 0.6           # Reduced from 0.8
        
        # Frame zones - make tracking smoother
        self.center_threshold = 0.2      # Increased from 0.15 for wider "center" zone
        self.max_error = 0.4             # Limit maximum error for gentler turns
        
        # Tracking state
        self.last_image_x = None
        self.last_turn_cmd = 0.0
        self.last_track_time = 0.0
        self.track_timeout = 0.2
        
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

    def calculate_turn_command(self, image_x: float, current_time: float) -> float:
        """Calculate turn speed to face human."""
        # Update tracking state
        time_delta = current_time - self.last_track_time
        self.last_track_time = current_time
        
        # Calculate normalized error from image center (-1 to 1)
        image_center = 320
        normalized_error = (image_x - image_center) / image_center
        
        # Limit maximum error to prevent large turns
        if abs(normalized_error) > self.max_error:
            normalized_error = math.copysign(self.max_error, normalized_error)
            self.node.get_logger().info(
                f'Large turn limited: error capped at {normalized_error:.2f}'
            )
        
        # Basic proportional control with deadzone
        if abs(normalized_error) < self.center_threshold:
            turn_speed = 0.0  # In center zone - don't turn
            self.node.get_logger().info('Target centered - no turn needed')
        else:
            # Calculate base turn speed
            turn_speed = self.turn_p_gain * normalized_error
            
            # Ensure minimum speed if turning
            if abs(turn_speed) < self.min_rotation_speed:
                turn_speed = math.copysign(self.min_rotation_speed, normalized_error)
            
            # Gentle speed scaling
            scale_factor = 1.0 + (0.3 * abs(normalized_error))  # Max 1.3x scaling
            turn_speed *= scale_factor
            
            # Enforce maximum speed
            turn_speed = max(min(turn_speed, self.max_rotation_speed), 
                           -self.max_rotation_speed)
            
            self.node.get_logger().info(
                f'Turn: error={normalized_error:.2f}, speed={turn_speed:.2f}'
            )
        
        # Store state for next iteration
        self.last_image_x = image_x
        self.last_turn_cmd = turn_speed
        
        return turn_speed

    def get_avoidance_command(self, human_distance, human_angle, image_x=None):
        cmd = Twist()
        needs_escape = False
        current_time = time.time()
        
        # Check if we're in forward motion mode
        if self.is_moving_forward:
            if current_time - self.forward_motion_start < self.forward_motion_duration:
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
                        self.forward_motion_start = current_time
                        cmd.linear.x = 0.1
                    return cmd, needs_escape
                
        # Handle turning to face human
        if image_x is not None:
            cmd.angular.z = self.calculate_turn_command(image_x, current_time)
            
            # Don't back up during large turns
            if abs(cmd.angular.z) > self.max_rotation_speed * 0.7:
                self.node.get_logger().info('Large turn - pausing backup')
                return cmd, False
                
        else:  # No current human detection
            # Decay any existing turn
            time_since_track = current_time - self.last_track_time
            if time_since_track < self.track_timeout and self.last_turn_cmd != 0:
                decay_factor = 1.0 - (time_since_track / self.track_timeout)
                cmd.angular.z = self.last_turn_cmd * decay_factor
                self.node.get_logger().info(f'Decay turn: {cmd.angular.z:.2f}')
            else:
                cmd.angular.z = 0.0
                self.last_turn_cmd = 0.0
                self.last_image_x = None
        
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