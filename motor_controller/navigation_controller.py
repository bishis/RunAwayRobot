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
    TURNING = 2
    AVOIDING = 3
    STOPPED = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters - adjusted for better responsiveness
        self.declare_parameter('safety_radius', 0.5)  # Increased from 0.3
        self.declare_parameter('detection_distance', 1.0)  # Increased from 0.5
        
        self.safety_radius = self.get_parameter('safety_radius').value
        self.detection_distance = self.get_parameter('detection_distance').value
        
        # Robot state
        self.state = RobotState.STOPPED
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.target_yaw = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Control loop timer (10Hz)
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')
    
    def check_lidar_sectors(self):
        """Process LIDAR data into sectors and check for obstacles."""
        if not self.latest_scan:
            return None
            
        # Adjusted sectors for better coverage (adding 180° to align with robot's front)
        sectors = {
            'front': (150, 210),        # Front is now 180° ±30°
            'front_left': (210, 240),   # Front-left is now 210° to 240°
            'front_right': (120, 150),  # Front-right is now 120° to 150°
            'left': (240, 270),         # Left is now around 270°
            'right': (90, 120)          # Right is now around 90°
        }
        
        sector_data = {}
        for sector_name, (start, end) in sectors.items():
            # Convert angles to array indices
            start_idx = int((start * len(self.latest_scan.ranges) / 360))
            end_idx = int((end * len(self.latest_scan.ranges) / 360))
            
            # Get ranges for this sector
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
                min_dist = min(ranges)
                sector_data[sector_name] = min_dist
                self.get_logger().debug(f"{sector_name}: {min_dist:.2f}m")
            else:
                sector_data[sector_name] = float('inf')
        
        return sector_data
    
    def determine_movement(self, sector_data):
        """Determine movement based on LIDAR sectors."""
        if not sector_data:
            return (0.0, 0.0)  # Stop if no data
            
        # Debug print
        self.get_logger().info(f"Sector distances: {sector_data}")
        
        # More aggressive obstacle avoidance
        if sector_data['front'] < self.safety_radius * 1.5:  # Added buffer
            self.get_logger().warn(f"Front obstacle at {sector_data['front']:.2f}m")
            # If front is blocked, check sides
            if sector_data['front_left'] > sector_data['front_right']:
                self.get_logger().info("Turning left - obstacle avoidance")
                return (0.0, 1.0)  # Sharp left turn
            else:
                self.get_logger().info("Turning right - obstacle avoidance")
                return (0.0, -1.0)  # Sharp right turn
        
        # Preventive turning when obstacles are near
        elif sector_data['front_left'] < self.safety_radius:
            self.get_logger().info("Preventive right turn")
            return (0.3, -1.0)  # Slower forward, strong right turn
        
        elif sector_data['front_right'] < self.safety_radius:
            self.get_logger().info("Preventive left turn")
            return (0.3, 1.0)  # Slower forward, strong left turn
        
        # If path is clear but obstacles nearby, make gentle adjustments
        elif sector_data['front_left'] < self.detection_distance:
            return (0.8, -0.5)  # Maintain speed, gentle right
        
        elif sector_data['front_right'] < self.detection_distance:
            return (0.8, 0.5)  # Maintain speed, gentle left
        
        # If path is completely clear, go forward
        else:
            self.get_logger().info("Path clear, moving forward")
            return (1.0, 0.0)
    
    def control_loop(self):
        """Main control loop."""
        if not self.current_pose or not self.latest_scan:
            self.get_logger().warn('Waiting for sensor data...')
            return
        
        # Process LIDAR data
        sector_data = self.check_lidar_sectors()
        if not sector_data:
            self.get_logger().warn('No valid LIDAR data')
            self.stop_robot()
            return
        
        # Determine movement
        linear_x, angular_z = self.determine_movement(sector_data)
        
        # Add emergency stop check
        if min(sector_data.values()) < self.safety_radius * 0.5:  # Very close obstacle
            self.get_logger().warn("Emergency stop - obstacle too close!")
            self.stop_robot()
            return
        
        # Send command to robot
        self.send_velocity_command(linear_x, angular_z)
        
        # Log state with more detail
        self.get_logger().info(
            f'Command: linear={linear_x:.1f}, angular={angular_z:.1f}\n'
            f'Distances - Front: {sector_data["front"]:.2f}m, '
            f'FL: {sector_data["front_left"]:.2f}m, '
            f'FR: {sector_data["front_right"]:.2f}m'
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