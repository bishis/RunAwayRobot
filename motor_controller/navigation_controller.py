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
    TURNING = 2
    AVOIDING = 3
    STOPPED = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('safety_radius', 0.3)
        self.declare_parameter('detection_distance', 0.5)
        self.declare_parameter('leg_length', 2.0)
        
        self.safety_radius = self.get_parameter('safety_radius').value
        self.detection_distance = self.get_parameter('detection_distance').value
        self.leg_length = self.get_parameter('leg_length').value
        
        # Robot state
        self.state = RobotState.STOPPED
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.start_position = None
        self.target_yaw = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Control loop timer (10Hz)
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')
    
    def map_callback(self, msg):
        """Process map updates."""
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_width = msg.info.width
        self.map_height = msg.info.height
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates."""
        if not self.map_data:
            return None, None
        
        mx = int((x - self.map_origin.position.x) / self.map_resolution)
        my = int((y - self.map_origin.position.y) / self.map_resolution)
        
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            return mx, my
        return None, None
    
    def check_path_to_point(self, start_x, start_y, end_x, end_y):
        """Check if path to point is clear using map data."""
        if not self.map_data:
            return False
            
        # Convert points to map coordinates
        sx, sy = self.world_to_map(start_x, start_y)
        ex, ey = self.world_to_map(end_x, end_y)
        
        if None in (sx, sy, ex, ey):
            return False
            
        # Simple line checking
        points = self.bresenham_line(sx, sy, ex, ey)
        for x, y in points:
            idx = y * self.map_width + x
            if idx < len(self.map_data.data):
                if self.map_data.data[idx] > 50:  # Occupied cell
                    return False
        return True
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Generate points along a line using Bresenham's algorithm."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points
    
    def check_immediate_obstacles(self):
        """Check LIDAR data for immediate obstacles."""
        if not self.latest_scan:
            return True
            
        # Check front area
        front_angles = range(-30, 31)
        for angle in front_angles:
            idx = (angle + 360) % 360
            if idx < len(self.latest_scan.ranges):
                distance = self.latest_scan.ranges[idx]
                if 0.1 < distance < self.safety_radius:
                    self.get_logger().warn(f'Immediate obstacle at {distance}m')
                    return True
        return False
    
    def find_clear_direction(self):
        """Find the clearest direction to move using LIDAR data."""
        if not self.latest_scan:
            return None
            
        # Divide scan into sectors
        sector_size = 30
        num_sectors = 360 // sector_size
        sectors = [[] for _ in range(num_sectors)]
        
        for i, range_val in enumerate(self.latest_scan.ranges):
            if 0.1 < range_val < float('inf'):
                sector_idx = i // sector_size
                sectors[sector_idx].append(range_val)
        
        # Find sector with maximum average distance
        max_dist = 0
        best_sector = 0
        for i, sector in enumerate(sectors):
            if sector:
                avg_dist = sum(sector) / len(sector)
                if avg_dist > max_dist:
                    max_dist = avg_dist
                    best_sector = i
        
        return best_sector * sector_size
    
    def control_loop(self):
        """Main control loop."""
        if not self.current_pose or not self.latest_scan:
            self.get_logger().warn('Waiting for sensor data...')
            return
            
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Initialize start position if needed
        if self.start_position is None:
            self.start_position = (current_x, current_y)
            self.state = RobotState.EXPLORING
            self.get_logger().info('Starting exploration')
        
        # Check for immediate obstacles
        if self.check_immediate_obstacles():
            self.state = RobotState.AVOIDING
        
        # State machine
        if self.state == RobotState.AVOIDING:
            # Find clear direction
            clear_direction = self.find_clear_direction()
            if clear_direction is not None:
                angle_diff = clear_direction - math.degrees(current_yaw)
                while angle_diff > 180: angle_diff -= 360
                while angle_diff < -180: angle_diff += 360
                
                if abs(angle_diff) > 10:
                    # Turn towards clear direction
                    turn_speed = 1.0 if angle_diff > 0 else -1.0
                    self.send_velocity_command(0.0, turn_speed)
                else:
                    # Clear path found, resume exploring
                    self.state = RobotState.EXPLORING
            else:
                self.stop_robot()
                
        elif self.state == RobotState.EXPLORING:
            # Check if current path is clear
            target_x = current_x + math.cos(current_yaw) * self.detection_distance
            target_y = current_y + math.sin(current_yaw) * self.detection_distance
            
            if self.check_path_to_point(current_x, current_y, target_x, target_y):
                self.send_velocity_command(1.0, 0.0)
            else:
                self.state = RobotState.AVOIDING
        
        self.get_logger().info(
            f'State: {self.state.name}, '
            f'Position: ({current_x:.2f}, {current_y:.2f}), '
            f'Yaw: {math.degrees(current_yaw):.1f}°'
        )
    
    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to the robot."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f'Sent velocity command - linear: {linear_x}, angular: {angular_z}')
    
    def stop_robot(self):
        """Stop the robot."""
        self.send_velocity_command(0.0, 0.0)
        self.get_logger().info('Stopping robot')
    
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