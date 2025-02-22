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
        self.min_safe_distance = 1.25  # Start backing up at 1m
        self.critical_distance = 0.4   # Request escape at 0.3m
        self.ready_to_flee_distance = 0.5  # Distance to enter ready-to-flee mode
        self.max_backup_speed = 0.15   # Increase backup speed
        
        # Tracking parameters
        self.max_rotation_speed = 0.06
        
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

    def check_rear_safety(self) -> tuple[str, float]:
        """
        Check if there's a critical distance behind the robot.
        Returns (status, min_distance) where status is:
        - 'safe': Normal operation
        - 'ready': Ready to flee (close to wall)
        - 'critical': Critical distance detected
        """
        if self.latest_scan is None:
            self.node.get_logger().warn('No scan data available for rear monitoring')
            return 'safe', float('inf')  # Assume safe if no data
        
        try:
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
                
                # Check distances
                if rear_distance < self.critical_distance:
                    return 'critical', rear_distance
                elif rear_distance < self.ready_to_flee_distance:
                    return 'ready', rear_distance
                
                return 'safe', rear_distance
            
            return 'safe', float('inf')
        
        except Exception as e:
            self.node.get_logger().error(f'Error checking rear safety: {str(e)}')
            return 'safe', float('inf')

    def get_avoidance_command(self, human_distance, human_angle, image_x=None):
        """Calculate avoidance command based on human position"""
        cmd = Twist()
        
        # First check rear safety and get distance
        rear_status, rear_distance = self.check_rear_safety()
        
        # Handle critical rear distance
        if rear_status == 'critical':
            self.node.get_logger().error('EMERGENCY STOP - Critical rear distance!')
            cmd = Twist()  # Zero velocity command
            return cmd, True  # Return emergency stop and trigger escape
        
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
                cmd = Twist()
                
                # Check if we need to escape when in ready-to-flee mode
                if rear_status == 'ready' and human_distance < self.min_safe_distance:
                    self.node.get_logger().warn('Ready to flee and human too close - initiating escape!')
                    return cmd, True  # Trigger escape
                
                # Only allow backup when human is centered and we're not too close to wall
                if human_distance < self.min_safe_distance and rear_distance > self.ready_to_flee_distance:
                    # Calculate backup speed to maintain 0.5m from wall
                    target_distance = self.ready_to_flee_distance  # Stop at 0.5m from wall
                    distance_to_target = rear_distance - target_distance
                    
                    if distance_to_target > 0:  # Only back up if we have room
                        # Scale speed based on distance to target
                        speed_ratio = min(1.0, distance_to_target / 0.5)  # Full speed when >0.5m from target
                        backup_speed = self.max_backup_speed * speed_ratio
                        
                        cmd.linear.x = -backup_speed
                        self.node.get_logger().info(
                            f'Human centered - backing up at {backup_speed:.2f} m/s '
                            f'(wall distance: {rear_distance:.2f}m, target: {target_distance:.2f}m)'
                        )
                    else:
                        self.node.get_logger().info('At target distance from wall - holding position')
                
                return cmd, False
            
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


