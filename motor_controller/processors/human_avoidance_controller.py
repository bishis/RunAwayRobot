#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time
from .human_escape import HumanEscape
import tf_transformations
import random

def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

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
        
        # Get tf_buffer from node
        self.tf_buffer = self.node.tf_buffer
        self.wheel_speeds_pub = self.node.wheel_speeds_pub
        
        # Distance thresholds
        self.min_safe_distance = 1.25  # Start backing up at 1m
        self.ready_to_flee_distance = 0.45  # Distance to enter ready-to-flee mode
        self.max_backup_speed = 0.15   # Increase backup speed
        
        # Tracking parameters
        self.max_rotation_speed = 0.03
        
        # Frame zones - simpler tracking
        self.center_zone = 0.25  # Consider centered within ±25% of frame center
        
        # Add turn timeout tracking
        self.turn_start_time = None
        self.turn_timeout = 10.0  # 10 seconds max for turning
        
        # Tracking state
        self.last_image_x = None
        self.is_turning = False
        
        # Get latest scan data from node
        self.latest_scan = None
        if hasattr(node, 'latest_scan'):
            self.latest_scan = node.latest_scan
            
        # Add tracking for previous escape points
        self.previous_escape_points = []
        self.max_escape_history = 3
        self.min_escape_distance = 0.7  # Minimum distance between escape points

    def calculate_turn_command(self, robot_pose, human_x, human_y) -> float:
        """Calculate turn speed to face human based on robot pose and human position."""
        try:
            # Extract robot position and orientation
            robot_x = robot_pose.pose.position.x
            robot_y = robot_pose.pose.position.y
            
            # Get robot orientation as Euler angles
            quat = [
                robot_pose.pose.orientation.x,
                robot_pose.pose.orientation.y,
                robot_pose.pose.orientation.z,
                robot_pose.pose.orientation.w
            ]
            _, _, robot_yaw = tf_transformations.euler_from_quaternion(quat)
            
            # Calculate angle to human
            dx = human_x - robot_x
            dy = human_y - robot_y
            desired_yaw = math.atan2(dy, dx)
            
            # Use the turn_to_angle function to get the Twist command
            cmd = self.turn_to_angle(desired_yaw)
            turn_speed = cmd.angular.z
            
            # Calculate the angle difference (shortest path)
            angle_diff = normalize_angle(desired_yaw - robot_yaw)
            
            # Check if human is in center zone (robot is facing human)
            if abs(angle_diff) <= 0.32:  # ~17.2 degrees tolerance
                self.node.get_logger().info('Robot facing human - stopping turn')
                self.is_turning = False  # Set turning flag to False when facing human
                return 0.0
            
            # Set turning flag based on whether we need to turn
            self.is_turning = abs(turn_speed) > 0.02
            
            self.node.get_logger().info(
                f'Turning to face human: angle_diff={angle_diff:.2f}rad ({math.degrees(angle_diff):.1f}°), '
                f'speed={turn_speed:.2f}'
            )
            
            return turn_speed
            
        except Exception as e:
            self.node.get_logger().error(f'Error in calculate_turn_command: {str(e)}')
            return 0.0

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
            # Define rear arc from 100° to 240° for 140° total coverage
            rear_start_angle = 100 * math.pi/180  # 100 degrees
            rear_end_angle = 240 * math.pi/180    # 240 degrees
            
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
                if rear_distance < self.ready_to_flee_distance:
                    return 'ready', rear_distance
                
                return 'safe', rear_distance
            
            return 'safe', float('inf')
        
        except Exception as e:
            self.node.get_logger().error(f'Error checking rear safety: {str(e)}')
            return 'safe', float('inf')

    def get_avoidance_command(self, human_distance, human_angle, robot_pose=None, human_pos=None):
        """Calculate avoidance command based on human position"""
        cmd = Twist()
        
        # First check rear safety and get distance
        rear_status, rear_distance = self.check_rear_safety()

        if rear_status == 'critical':
            self.node.get_logger().warn('Critical distance detected - initiating escape!')
            self.is_turning = False  # Reset turning flag
            return cmd, True  # Trigger escape
        
        # Handle turning to face human (using robot pose and human position)
        if robot_pose is not None and human_pos is not None:
            human_x, human_y = human_pos
            
            # Calculate turn command
            cmd.angular.z = self.calculate_turn_command(robot_pose, human_x, human_y)
            
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
                            f'Robot facing human - backing up at {backup_speed:.2f} m/s '
                            f'(wall distance: {rear_distance:.2f}m, target: {target_distance:.2f}m)'
                        )
                    else:
                        self.node.get_logger().info('At target distance from wall - holding position')
                
                return cmd, False
            
        else:  # No human detected or missing pose data
            # Stop all motion
            stop_cmd = Twist()
            self.is_turning = False  # Reset turning flag
            self.last_image_x = None
            return stop_cmd, False

    def plan_escape(self, add_randomness=False):
        """Plan an escape path away from a human"""
        if self.last_human_position is None:
            return None
        
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
            
            # Evaluate the point against previous escape points
            new_x = escape_point.pose.position.x
            new_y = escape_point.pose.position.y
            
            # Calculate distances to previous escape points
            distances_to_previous = []
            directions_to_avoid = []
            
            for prev_point in self.previous_escape_points:
                prev_x = prev_point.pose.position.x
                prev_y = prev_point.pose.position.y
                
                # Calculate distance and direction to previous point
                dx = prev_x - new_x
                dy = prev_y - new_y
                dist = math.sqrt(dx*dx + dy*dy)
                
                # Record distance
                distances_to_previous.append(dist)
                
                # If point is close to previous one, record its direction to avoid
                if dist < 1.0:
                    angle = math.atan2(dy, dx)
                    directions_to_avoid.append(angle)
            
            # If we have previous similar escape points, modify the escape direction
            if directions_to_avoid or add_randomness:
                # Start with current human-to-robot vector
                robot_x = self.node.current_pose.pose.position.x
                robot_y = self.node.current_pose.pose.position.y
                human_x, human_y = self.last_human_position
                
                # Base escape direction (away from human)
                base_dx = robot_x - human_x
                base_dy = robot_y - human_y
                base_angle = math.atan2(base_dy, base_dx)
                
                # Try different angles, starting with highest deviation
                best_angle = base_angle
                best_score = -float('inf')
                
                # Try angles up to 120 degrees in either direction from base angle
                for angle_offset in np.linspace(-2.0, 2.0, 9):  # -120° to +120° in 9 steps
                    test_angle = base_angle + angle_offset
                    
                    # Calculate score based on:
                    # 1. How different this angle is from previous failed attempts
                    # 2. How close it is to the ideal escape direction
                    
                    # Start with base score
                    angle_score = 1.0
                    
                    # Penalize angles close to previous attempts
                    for avoid_angle in directions_to_avoid:
                        angle_diff = abs(normalize_angle(test_angle - avoid_angle))
                        # Penalize more if angle is similar to a failed attempt
                        if angle_diff < 0.5:  # About 30 degrees
                            angle_score -= (0.5 - angle_diff) * 2.0
                    
                    # Penalize deviation from ideal escape direction
                    ideal_deviation = abs(normalize_angle(test_angle - base_angle))
                    angle_score -= ideal_deviation * 0.5  # Less penalty for deviation
                    
                    # Add randomness if requested
                    if add_randomness:
                        angle_score += random.uniform(0, 0.5)
                    
                    if angle_score > best_score:
                        best_score = angle_score
                        best_angle = test_angle
                
                # Use the best angle to generate a new escape point
                distance = 2.0  # 2 meters away
                new_x = human_x + distance * math.cos(best_angle)
                new_y = human_y + distance * math.sin(best_angle)
                
                # Create a new escape point
                escape_point.pose.position.x = new_x
                escape_point.pose.position.y = new_y
                
                self.node.get_logger().info(
                    f'Modified escape point to avoid previous paths: ({new_x:.2f}, {new_y:.2f})'
                )
            
            # Add current point to history
            self.previous_escape_points.append(escape_point)
            if len(self.previous_escape_points) > self.max_escape_history:
                self.previous_escape_points.pop(0)
            
            # Final check: Ensure this isn't exactly the same as a previous escape point
            new_x = escape_point.pose.position.x
            new_y = escape_point.pose.position.y
            
            # Check for exact matches with previous points
            for prev_point in self.previous_escape_points:
                prev_x = prev_point.pose.position.x
                prev_y = prev_point.pose.position.y
                
                # If almost exactly the same point, add mandatory randomness
                if abs(new_x - prev_x) < 0.1 and abs(new_y - prev_y) < 0.1:
                    self.node.get_logger().warn('Generated identical escape point - forcing randomness')
                    
                    # Get current direction and add significant randomness
                    robot_x = self.node.current_pose.pose.position.x
                    robot_y = self.node.current_pose.pose.position.y
                    human_x, human_y = self.last_human_position
                    
                    # Base direction but with large offset
                    random_angle = random.uniform(-1.0, 1.0)  # Up to 60° in either direction
                    base_angle = math.atan2(robot_y - human_y, robot_x - human_x) + random_angle
                    
                    # Set distance with randomness
                    distance = random.uniform(1.5, 2.5)
                    
                    # Generate new point
                    new_x = human_x + distance * math.cos(base_angle)
                    new_y = human_y + distance * math.sin(base_angle)
                    
                    # Update escape point
                    escape_point.pose.position.x = new_x
                    escape_point.pose.position.y = new_y
                    
                    self.node.get_logger().info(
                        f'Forced randomized escape point: ({new_x:.2f}, {new_y:.2f})'
                    )
                    break
            
            return escape_point
        else:
            self.node.get_logger().error('Failed to find escape point!')
            return None

    def turn_to_angle(self, target_angle):
        """
        Calculate speeds needed to turn to a specific angle.
        Returns Twist message with rotation speeds and the time spent turning.
        """
        try:
            # Get current robot orientation from tf
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Extract current yaw from quaternion
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            _, _, current_yaw = tf_transformations.euler_from_quaternion(quaternion)
            
            # Calculate angle difference and normalize to [-pi, pi]
            angle_diff = normalize_angle(target_angle - current_yaw)
            
            # Calculate rotation speed based on angle difference
            MAX_ROTATION_SPEED = 0.3  # rad/s
            MIN_ROTATION_SPEED = 0.04  # rad/s
            ANGLE_THRESHOLD = 0.1  # radians
            
            if abs(angle_diff) < ANGLE_THRESHOLD:
                # Close enough to target angle
                cmd = Twist()
                cmd.angular.z = 0.0  # Stop turning
                return cmd
            
            # Scale rotation speed based on angle difference
            rotation_speed = max(
                MIN_ROTATION_SPEED,
                min(MAX_ROTATION_SPEED, abs(angle_diff))
            )
            
            # Set direction based on shortest rotation
            if angle_diff > 0:
                cmd = Twist()
                cmd.angular.z = rotation_speed
            else:
                cmd = Twist()
                cmd.angular.z = -rotation_speed
            
            self.node.get_logger().info(
                f'Turning to angle {target_angle:.2f}: '
                f'current={current_yaw:.2f}, '
                f'diff={angle_diff:.2f}, '
                f'speed={cmd.angular.z:.2f}'
            )
            
            return cmd
        
        except Exception as e:
            self.node.get_logger().error(f'Error calculating turn angle: {str(e)}')
            return Twist()  # Return zero speeds on error
