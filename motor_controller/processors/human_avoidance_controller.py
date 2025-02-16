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
        
        # Improved tracking parameters
        self.min_rotation_speed = 0.3
        self.max_rotation_speed = 0.75
        self.turn_p_gain = 1.2        # Increased from 0.8 for more responsive tracking
        
        # Frame zones
        self.frame_width = 640
        self.center_threshold = 0.3    # Narrower center zone - track more actively
        self.edge_threshold = 0.05     # Smaller edge zone
        self.tracking_deadzone = 0.1   # Small deadzone to prevent tiny corrections
        
        # Smoother transitions
        self.min_turn_duration = 0.2   # Shorter minimum turn time
        self.last_turn_time = 0.0
        self.last_turn_direction = 0
        self.last_error = 0.0         # For derivative control
        
        # Get latest scan data from node
        self.latest_scan = None
        if hasattr(node, 'latest_scan'):
            self.latest_scan = node.latest_scan
            
        # Add tracking parameters
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
        #self.node.create_timer(0.2, self.monitor_rear_distance)  # 5Hz updates
        
    def calculate_avoidance(self, human_x: float, distance: float) -> tuple[float, float]:
        """Calculate avoidance velocities with improved tracking"""
        current_time = time.time()
        
        # Normalize x position to [-1, 1]
        x_normalized = (2.0 * human_x / self.frame_width) - 1.0
        
        # Ignore humans at the very edges
        if abs(x_normalized) > (1.0 - self.edge_threshold):
            return 0.0, 0.0
            
        # Calculate error
        error = x_normalized
        
        # Apply deadzone to prevent tiny oscillations
        if abs(error) < self.tracking_deadzone:
            error = 0.0
            
        # PD control for smoother tracking
        error_derivative = (error - self.last_error) / 0.1  # Assume 10Hz update rate
        self.last_error = error
        
        # Calculate base turn speed
        if abs(error) < self.center_threshold:
            # In center zone - small corrections only
            turn_speed = 0.3 * error
        else:
            # Outside center - more aggressive tracking
            turn_speed = self.turn_p_gain * error + 0.2 * error_derivative
            
        # Apply speed limits
        turn_speed = max(min(turn_speed, self.max_rotation_speed), -self.max_rotation_speed)
        
        # Maintain turn direction for minimum duration to prevent jitter
        if current_time - self.last_turn_time > self.min_turn_duration:
            if abs(turn_speed) > self.min_rotation_speed:
                self.last_turn_time = current_time
                self.last_turn_direction = math.copysign(1.0, turn_speed)
        else:
            # Keep turning in the same direction
            if abs(turn_speed) > 0:
                turn_speed = self.last_turn_direction * max(abs(turn_speed), self.min_rotation_speed)
                
        # Calculate linear speed based on distance with smoother transitions
        linear_speed = 0.0
        if distance < self.critical_distance:
            linear_speed = -self.max_backup_speed
        elif distance < self.min_safe_distance:
            # Smoother backup speed transition
            dist_ratio = (distance - self.critical_distance) / (self.min_safe_distance - self.critical_distance)
            linear_speed = -self.max_backup_speed * (1.0 - dist_ratio**2)
            
        return linear_speed, turn_speed

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
                    if rear_distance < 0.2:  # 20cm safety threshold
                        self.node.get_logger().warn(
                            f'Wall too close behind ({rear_distance:.2f}m), moving forward instead'
                        )
                        cmd.linear.x = 0.1  # Small forward motion
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