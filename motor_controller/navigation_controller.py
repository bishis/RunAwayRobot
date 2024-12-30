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
    FORWARD = 1
    ROTATING = 2    # New state for in-place rotation
    AVOIDING = 3
    STOPPED = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('safety_radius', 0.5)
        self.declare_parameter('detection_distance', 1.0)
        
        self.safety_radius = self.get_parameter('safety_radius').value
        self.detection_distance = self.get_parameter('detection_distance').value
        
        # Robot state
        self.state = RobotState.FORWARD
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.target_rotation = None  # Target angle to rotate to
        self.rotation_direction = 1  # 1 for right, -1 for left
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigation controller initialized')
    
    def check_lidar_sectors(self):
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
                    if 0.1 < range_val < 5.0:
                        ranges.append(range_val)
            
            if ranges:
                sector_data[sector_name] = min(ranges)
            else:
                sector_data[sector_name] = float('inf')
        
        return sector_data
    
    def find_best_rotation(self, sector_data):
        """Find the best direction to rotate based on LIDAR data."""
        # Check which direction has more space
        left_space = min(sector_data['left'], sector_data['front_left'])
        right_space = min(sector_data['right'], sector_data['front_right'])
        
        if right_space > left_space:
            return -1  # Rotate clockwise (right)
        return 1      # Rotate counter-clockwise (left)
    
    def determine_movement(self, sector_data):
        if not sector_data:
            return (0.0, 0.0)
        
        self.get_logger().info(f"State: {self.state.name}, Sector distances: {sector_data}")
        
        # Check if path is blocked
        front_blocked = sector_data['front'] < self.safety_radius * 1.5
        left_blocked = sector_data['front_left'] < self.safety_radius
        right_blocked = sector_data['front_right'] < self.safety_radius
        
        if self.state == RobotState.FORWARD:
            if front_blocked or left_blocked or right_blocked:
                self.state = RobotState.ROTATING
                self.rotation_direction = self.find_best_rotation(sector_data)
                self.get_logger().info(f"Obstacle detected, starting rotation {'right' if self.rotation_direction < 0 else 'left'}")
                return (0.0, self.rotation_direction * 1.0)  # Start rotating
            return (1.0, 0.0)  # Continue forward
            
        elif self.state == RobotState.ROTATING:
            # Keep rotating until we find a clear path
            if not front_blocked and not left_blocked and not right_blocked:
                self.state = RobotState.FORWARD
                self.get_logger().info("Clear path found, moving forward")
                return (1.0, 0.0)
            return (0.0, self.rotation_direction * 1.0)  # Continue rotating
            
        return (0.0, 0.0)  # Default stop
    
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