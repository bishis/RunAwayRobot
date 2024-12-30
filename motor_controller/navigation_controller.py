#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from enum import Enum

class RobotState(Enum):
    EXPLORING = 1
    ROTATING = 2
    FOLLOWING_PATH = 3
    STOPPED = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('safety_radius', 0.5)
        self.declare_parameter('detection_distance', 1.0)
        self.declare_parameter('look_ahead_distance', 1.5)  # Distance to look ahead for planning
        
        self.safety_radius = self.get_parameter('safety_radius').value
        self.detection_distance = self.get_parameter('detection_distance').value
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        
        # Robot state
        self.state = RobotState.EXPLORING
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.rotation_direction = 1
        self.last_turn_time = self.get_clock().now()
        self.unexplored_directions = list(range(0, 360, 45))  # Directions to explore (in degrees)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigation controller initialized')
    
    def find_best_direction(self, sector_data):
        """Find best direction considering both LIDAR and unexplored areas."""
        if not sector_data:
            return None
            
        # Get current yaw
        if not self.current_pose:
            return None
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        current_yaw_deg = math.degrees(current_yaw) % 360
        
        # Score each potential direction
        best_score = -1
        best_direction = None
        
        for direction in self.unexplored_directions[:]:  # Copy list for iteration
            # Convert to robot's frame
            relative_direction = (direction - current_yaw_deg) % 360
            
            # Check if this direction is clear
            if self.is_direction_clear(relative_direction, sector_data):
                # Calculate score based on:
                # 1. Distance to obstacles
                # 2. How different it is from current direction
                # 3. Whether it's unexplored
                distance_score = self.get_distance_score(relative_direction, sector_data)
                turn_score = 1.0 - abs(relative_direction - 180) / 180.0  # Prefer forward directions
                
                total_score = distance_score * 0.6 + turn_score * 0.4
                
                if total_score > best_score:
                    best_score = total_score
                    best_direction = direction
        
        return best_direction
    
    def is_direction_clear(self, direction, sector_data):
        """Check if a direction is clear of obstacles."""
        # Map direction to sectors
        if 150 <= direction <= 210:  # Front
            return sector_data['front'] > self.safety_radius * 1.5
        elif 210 < direction <= 270:  # Left
            return sector_data['left'] > self.safety_radius
        elif 90 <= direction < 150:  # Right
            return sector_data['right'] > self.safety_radius
        return False
    
    def get_distance_score(self, direction, sector_data):
        """Calculate score based on distance to obstacles."""
        # Get relevant distance based on direction
        if 150 <= direction <= 210:
            dist = sector_data['front']
        elif 210 < direction <= 270:
            dist = sector_data['left']
        elif 90 <= direction < 150:
            dist = sector_data['right']
        else:
            return 0.0
        
        # Normalize distance score
        return min(1.0, dist / self.look_ahead_distance)
    
    def determine_movement(self, sector_data):
        if not sector_data:
            return (0.0, 0.0)
        
        # Check if path is blocked
        front_blocked = sector_data['front'] < self.safety_radius * 1.5
        left_blocked = sector_data['front_left'] < self.safety_radius
        right_blocked = sector_data['front_right'] < self.safety_radius
        
        self.get_logger().info(f"Distances - Front: {sector_data['front']:.2f}m, "
                              f"FL: {sector_data['front_left']:.2f}m, "
                              f"FR: {sector_data['front_right']:.2f}m")
        
        if self.state == RobotState.EXPLORING:
            # If path is blocked, find best direction
            if front_blocked or left_blocked or right_blocked:
                best_direction = self.find_best_direction(sector_data)
                
                if best_direction is not None:
                    current_yaw = math.degrees(self.get_yaw_from_quaternion(self.current_pose.orientation))
                    angle_diff = (best_direction - current_yaw) % 360
                    if angle_diff > 180:
                        angle_diff -= 360
                    
                    # If significant turn needed
                    if abs(angle_diff) > 20:
                        self.state = RobotState.ROTATING
                        self.rotation_direction = 1 if angle_diff > 0 else -1
                        self.get_logger().info(f"Starting rotation {self.rotation_direction}")
                        return (0.0, self.rotation_direction * 1.0)
                    
                    # Small adjustment while moving
                    turn_rate = np.clip(angle_diff / 45.0, -0.5, 0.5)
                    return (0.8, turn_rate)
                
                # No good direction found, rotate in place
                self.state = RobotState.ROTATING
                self.rotation_direction = 1
                return (0.0, 1.0)
            
            # Path is clear, move forward
            return (1.0, 0.0)
            
        elif self.state == RobotState.ROTATING:
            # Keep rotating until we find a clear path
            if not front_blocked and not left_blocked and not right_blocked:
                self.state = RobotState.EXPLORING
                current_yaw = math.degrees(self.get_yaw_from_quaternion(self.current_pose.orientation))
                self.update_explored_directions(current_yaw)
                self.get_logger().info("Found clear path, moving forward")
                return (1.0, 0.0)
            return (0.0, self.rotation_direction * 1.0)
        
        return (0.0, 0.0)
    
    def update_explored_directions(self, current_yaw):
        """Update list of unexplored directions."""
        # Remove directions that have been explored
        self.unexplored_directions = [d for d in self.unexplored_directions 
                                    if abs((d - current_yaw) % 360) > 45]
        
        # Reset list if all directions explored
        if not self.unexplored_directions:
            self.unexplored_directions = list(range(0, 360, 45))
    
    def control_loop(self):
        if not self.current_pose or not self.latest_scan:
            return
        
        sector_data = self.check_lidar_sectors()
        if not sector_data:
            self.stop_robot()
            return
        
        # Emergency stop check
        if min(sector_data.values()) < self.safety_radius * 0.5:
            self.get_logger().warn("Emergency stop - obstacle too close!")
            self.stop_robot()
            return
        
        linear_x, angular_z = self.determine_movement(sector_data)
        self.send_velocity_command(linear_x, angular_z)
        
        self.get_logger().info(
            f'State: {self.state.name}, Command: linear={linear_x:.1f}, angular={angular_z:.1f}'
        )
    
    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to the robot."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot."""
        self.send_velocity_command(0.0, 0.0)
    
    def map_callback(self, msg):
        """Process map updates."""
        self.map_data = msg
    
    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg
    
    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def check_lidar_sectors(self):
        """Process LIDAR data into sectors and check for obstacles."""
        if not self.latest_scan:
            return None
        
        sectors = {
            'front': (150, 210),      # Front is 180° ±30°
            'front_left': (210, 240),
            'front_right': (120, 150),
            'left': (240, 270),
            'right': (90, 120)
        }
        
        sector_data = {}
        for sector_name, (start, end) in sectors.items():
            start_idx = int((start * len(self.latest_scan.ranges) / 360))
            end_idx = int((end * len(self.latest_scan.ranges) / 360))
            
            ranges = []
            if start_idx <= end_idx:
                indices = range(start_idx, end_idx + 1)
            else:
                indices = list(range(start_idx, len(self.latest_scan.ranges))) + list(range(0, end_idx + 1))
            
            for i in indices:
                if i < len(self.latest_scan.ranges):
                    range_val = self.latest_scan.ranges[i]
                    if 0.1 < range_val < 5.0:  # Filter valid ranges
                        ranges.append(range_val)
            
            if ranges:
                sector_data[sector_name] = min(ranges)
                self.get_logger().debug(f"{sector_name}: {sector_data[sector_name]:.2f}m")
            else:
                sector_data[sector_name] = float('inf')
        
        return sector_data

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 