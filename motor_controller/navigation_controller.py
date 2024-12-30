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
    AVOIDING_LEFT = 2
    AVOIDING_RIGHT = 3
    RETURNING = 4
    STOPPED = 5

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
        self.original_heading = None
        self.avoidance_start_pose = None
        self.obstacle_side = None  # 'left' or 'right'
        self.parallel_distance = 0.4  # Distance to maintain from wall
        
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
            'right': (90, 120),
            'parallel_left': (225, 255),  # For parallel wall following
            'parallel_right': (105, 135)
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
    
    def determine_movement(self, sector_data):
        if not sector_data:
            return (0.0, 0.0)
        
        self.get_logger().info(f"State: {self.state.name}, Sector distances: {sector_data}")
        
        if self.state == RobotState.FORWARD:
            if sector_data['front'] < self.safety_radius * 1.5:
                # Decide which way to avoid based on more open space
                if sector_data['left'] > sector_data['right']:
                    self.state = RobotState.AVOIDING_LEFT
                    self.obstacle_side = 'right'
                    self.avoidance_start_pose = self.current_pose
                    return (0.0, 1.0)  # Start turning left
                else:
                    self.state = RobotState.AVOIDING_RIGHT
                    self.obstacle_side = 'left'
                    self.avoidance_start_pose = self.current_pose
                    return (0.0, -1.0)  # Start turning right
            return (1.0, 0.0)  # Continue forward
            
        elif self.state in [RobotState.AVOIDING_LEFT, RobotState.AVOIDING_RIGHT]:
            # Wall following behavior
            is_avoiding_left = self.state == RobotState.AVOIDING_LEFT
            parallel_sector = 'parallel_left' if is_avoiding_left else 'parallel_right'
            front_side = 'front_left' if is_avoiding_left else 'front_right'
            
            # Check if we can return to original heading
            if self.can_return_to_path(sector_data):
                self.state = RobotState.RETURNING
                return (0.0, -1.0 if is_avoiding_left else 1.0)
            
            # Wall following adjustments
            parallel_dist = sector_data[parallel_sector]
            error = parallel_dist - self.parallel_distance
            turn_adjust = np.clip(error * 2.0, -1.0, 1.0)  # P controller
            
            # Stronger turn if front is blocked
            if sector_data[front_side] < self.safety_radius:
                turn_adjust = 1.0 if is_avoiding_left else -1.0
            
            return (0.5, turn_adjust)  # Move forward while adjusting
            
        elif self.state == RobotState.RETURNING:
            # Check if we're back on original heading
            if self.is_aligned_with_original_heading():
                self.state = RobotState.FORWARD
                return (1.0, 0.0)
            return (0.3, -1.0 if self.obstacle_side == 'right' else 1.0)
        
        return (0.0, 0.0)  # Default stop
    
    def can_return_to_path(self, sector_data):
        """Check if we can return to original path."""
        if not self.avoidance_start_pose:
            return False
            
        # Check if we've moved past the obstacle
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        start_x = self.avoidance_start_pose.position.x
        start_y = self.avoidance_start_pose.position.y
        
        # Calculate distance moved perpendicular to original direction
        dist_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
        
        # Check if we've moved enough and front is clear
        return (dist_moved > 1.0 and 
                sector_data['front'] > self.detection_distance and
                sector_data['front_left'] > self.safety_radius and
                sector_data['front_right'] > self.safety_radius)
    
    def is_aligned_with_original_heading(self):
        """Check if we're aligned with original heading."""
        if self.original_heading is None:
            return True
            
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_diff = abs(current_yaw - self.original_heading)
        return angle_diff < math.radians(10)  # Within 10 degrees
    
    def control_loop(self):
        if not self.current_pose or not self.latest_scan:
            return
        
        # Store original heading when starting
        if self.original_heading is None:
            self.original_heading = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
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