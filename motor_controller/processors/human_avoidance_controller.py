#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import time
from .human_escape import HumanEscape

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
        self.critical_distance = 0.4   # Request escape at 0.3m
        self.ready_to_flee_distance = 0.5  # Distance to enter ready-to-flee mode
        self.max_backup_speed = 0.15   # Increase backup speed
        
        # Tracking parameters
        self.max_rotation_speed = 0.035
        
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

    def turn_to_face_human(self, last_human_position):
        """Turn the robot to face the last known position of the human"""
        if last_human_position is not None:
            self.node.get_logger().info('Turning to face human...')
            
            human_x, human_y = last_human_position
            
            # Get current robot position
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Calculate angle to human
            dx = human_x - robot_x
            dy = human_y - robot_y
            angle_to_human = math.atan2(dy, dx)
            
            # Create a turn command
            turn_cmd = Twist()
            
            # Keep turning until facing the human
            while True:
                # Get current robot orientation
                current_transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                )
                q = current_transform.transform.rotation
                current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                # Calculate angle difference
                angle_diff = normalize_angle(current_yaw - angle_to_human)
                
                if abs(angle_diff) < 0.2:  # Adjust threshold
                    self.node.get_logger().info('Aligned with human position')
                    break
                
                # Proportional control for turn speed
                turn_speed = min(abs(angle_diff), self.max_rotation_speed)
                turn_cmd.angular.z = turn_speed if angle_diff > 0 else -turn_speed
                
                # Publish turn command
                self.wheel_speeds_pub.publish(turn_cmd)
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            # Stop turning
            self.wheel_speeds_pub.publish(Twist())
            self.node.get_logger().info('Stopped turning, ready to explore again')

    def add_human_to_slam_map(self, human_position, lidar_data):
        """
        Add the human position to the SLAM map using LiDAR data.
        
        Args:
            human_position: A tuple (x, y) representing the human's position in the map frame.
            lidar_data: The LiDAR data used to determine the size and shape of the human.
        """
        if not self.node.current_map:
            self.node.get_logger().warn('No map available for human mapping')
            return
        
        try:
            # Extract human coordinates
            human_x, human_y = human_position
            
            # Get map metadata
            resolution = self.node.current_map.info.resolution
            width = self.node.current_map.info.width
            height = self.node.current_map.info.height
            origin_x = self.node.current_map.info.origin.position.x
            origin_y = self.node.current_map.info.origin.position.y
            
            # Convert human position to grid coordinates
            grid_x = int((human_x - origin_x) / resolution)
            grid_y = int((human_y - origin_y) / resolution)
            
            # Check if coordinates are within map bounds
            if not (0 <= grid_x < width and 0 <= grid_y < height):
                self.node.get_logger().warn('Human position outside map bounds')
                return
            
            # Process LiDAR data to get human size and shape
            human_radius = self.process_lidar_data(lidar_data)
            
            # Convert radius to grid cells
            radius_cells = int(human_radius / resolution)
            
            # Update map data
            self.update_slam_map(grid_x, grid_y, radius_cells)
            
            self.node.get_logger().info(
                f'Added human at map coordinates ({grid_x}, {grid_y}) '
                f'with radius {human_radius:.2f}m ({radius_cells} cells)'
            )
            
        except Exception as e:
            self.node.get_logger().error(f'Error adding human to map: {str(e)}')

    def process_lidar_data(self, lidar_data):
        """
        Process LiDAR data to determine the size of the human.
        
        Args:
            lidar_data: The LiDAR scan data
            
        Returns:
            float: The estimated radius of the human in meters
        """
        if not lidar_data or not lidar_data.ranges:
            return 0.3  # Default human radius if no data
        
        try:
            # Define angular range to look for human (e.g., ±30 degrees)
            angle_range = math.pi/6  # 30 degrees
            
            # Get the center angle index
            center_idx = len(lidar_data.ranges) // 2
            
            # Calculate indices for the angular range
            points_per_radian = len(lidar_data.ranges) / (2 * math.pi)
            range_indices = int(angle_range * points_per_radian)
            
            # Get relevant scan points
            start_idx = max(0, center_idx - range_indices)
            end_idx = min(len(lidar_data.ranges), center_idx + range_indices)
            
            # Filter valid readings
            valid_readings = []
            for i in range(start_idx, end_idx):
                r = lidar_data.ranges[i]
                if lidar_data.range_min <= r <= lidar_data.range_max:
                    valid_readings.append(r)
            
            if not valid_readings:
                return 0.3  # Default if no valid readings
            
            # Calculate radius based on closest point and spread
            min_distance = min(valid_readings)
            spread = max(valid_readings) - min_distance
            
            # Estimate radius (using closest point and some spread)
            estimated_radius = max(0.2, min(0.4, (min_distance * math.tan(angle_range) + spread/2)))
            
            return estimated_radius
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing LIDAR data: {str(e)}')
            return 0.3  # Default radius on error

    def update_slam_map(self, grid_x, grid_y, radius_cells):
        """
        Update the SLAM map with the human's position and size.
        
        Args:
            grid_x: The x-coordinate in grid cells
            grid_y: The y-coordinate in grid cells
            radius_cells: The radius in grid cells
        """
        try:
            # Get map as 2D numpy array
            map_data = np.array(self.node.current_map.data).reshape(
                self.node.current_map.info.height,
                self.node.current_map.info.width
            )
            
            # Create a circle mask for the human
            y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
            mask = x*x + y*y <= radius_cells*radius_cells
            
            # Calculate bounds for the update region
            x_min = max(0, grid_x - radius_cells)
            x_max = min(self.node.current_map.info.width, grid_x + radius_cells + 1)
            y_min = max(0, grid_y - radius_cells)
            y_max = min(self.node.current_map.info.height, grid_y + radius_cells + 1)
            
            # Calculate mask bounds
            mask_x_start = radius_cells - (grid_x - x_min)
            mask_x_end = radius_cells + (x_max - grid_x)
            mask_y_start = radius_cells - (grid_y - y_min)
            mask_y_end = radius_cells + (y_max - grid_y)
            
            # Update occupancy values
            # Use 80 for human presence (higher than typical obstacles)
            update_region = map_data[y_min:y_max, x_min:x_max]
            update_mask = mask[mask_y_start:mask_y_end, mask_x_start:mask_x_end]
            update_region[update_mask] = 80
            
            # Convert back to 1D array and update map
            self.node.current_map.data = map_data.flatten().tolist()
            
            self.node.get_logger().info(
                f'Updated map region ({x_min}, {y_min}) to ({x_max}, {y_max})'
            )
            
        except Exception as e:
            self.node.get_logger().error(f'Error updating SLAM map: {str(e)}')


