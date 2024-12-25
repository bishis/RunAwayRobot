#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import math
import time

from .controllers.motor_controller import MotorController
from .processors.lidar_processor import LidarProcessor

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        
        # Initialize components
        self.motors = MotorController(
            left_pin=18,
            right_pin=12
        )
        self.lidar = LidarProcessor()
        
        # Movement state
        self.state = 'FORWARD'  # States: FORWARD, TURNING
        self.turn_direction = 'RIGHT'  # Alternates between LEFT and RIGHT
        self.turn_start_time = None
        self.forward_start_time = None
        self.turn_duration = 2.0  # Time to turn (seconds)
        self.forward_duration = 3.0  # Time to move forward (seconds)
        
        # Safety parameters
        self.safety_distance = 0.2  # 50cm safety distance
        self.critical_distance = 0.1  # 30cm critical distance
        self.scan_sectors = {
            'front': {'start': 350, 'end': 10, 'min_dist': float('inf')},
            'front_left': {'start': 10, 'end': 30, 'min_dist': float('inf')},
            'front_right': {'start': 330, 'end': 350, 'min_dist': float('inf')}
        }
        
        # Create subscriptions
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Add initial delay before starting
        time.sleep(2.0)
        
        # Create timer for movement control
        self.create_timer(0.1, self.move_robot)
        
        # Start moving forward
        self.start_forward()
        
    def start_forward(self):
        """Start moving forward."""
        self.state = 'FORWARD'
        self.forward_start_time = time.time()
        self.motors.set_speeds(1, 1)  # Move forward at 100% speed
        self.get_logger().info('Moving forward')
        
    def start_turning(self, direction=None):
        """Start turning in specified direction or use default pattern."""
        self.state = 'TURNING'
        self.turn_start_time = time.time()
        
        # If direction is specified, use it; otherwise use alternating pattern
        if direction:
            turn_direction = direction
        else:
            turn_direction = self.turn_direction
            self.turn_direction = 'LEFT' if turn_direction == 'RIGHT' else 'RIGHT'
        
        # Set turn speeds
        if turn_direction == 'RIGHT':
            self.motors.set_speeds(1, -1)
        else:
            self.motors.set_speeds(-1, 1)
            
        self.get_logger().info(f'Turning {turn_direction}')
        
    def check_sector(self, ranges, start_idx, end_idx):
        """Check minimum distance in a sector of the laser scan."""
        if start_idx > end_idx:  # Wrap around case (e.g., 350 to 10)
            sector_ranges = ranges[start_idx:] + ranges[:end_idx]
        else:
            sector_ranges = ranges[start_idx:end_idx]
            
        # Filter out invalid readings (zeros or very small values)
        valid_ranges = [r for r in sector_ranges if r > 0.1]
        return min(valid_ranges) if valid_ranges else float('inf')
        
    def move_robot(self):
        """Control robot movement pattern."""
        current_time = time.time()
        
        if self.state == 'FORWARD':
            # Check if we've been moving forward long enough
            if current_time - self.forward_start_time >= self.forward_duration:
                self.start_turning()
                
        elif self.state == 'TURNING':
            # Check if we've been turning long enough
            if current_time - self.turn_start_time >= self.turn_duration:
                self.start_forward()

    def lidar_callback(self, msg):
        """Process LIDAR data for obstacle detection."""
        # Update sector distances
        for sector in self.scan_sectors.values():
            sector['min_dist'] = self.check_sector(msg.ranges, sector['start'], sector['end'])
        
        # Log distances for debugging
        self.get_logger().debug(
            f"Distances - Front: {self.scan_sectors['front']['min_dist']:.2f}m, "
            f"Left: {self.scan_sectors['front_left']['min_dist']:.2f}m, "
            f"Right: {self.scan_sectors['front_right']['min_dist']:.2f}m"
        )
        
        # Check for obstacles and decide action
        front_dist = self.scan_sectors['front']['min_dist']
        left_dist = self.scan_sectors['front_left']['min_dist']
        right_dist = self.scan_sectors['front_right']['min_dist']
        
        if front_dist < self.critical_distance:
            # Emergency stop and turn
            self.get_logger().warn(f"Obstacle too close! Distance: {front_dist:.2f}m")
            self.motors.stop()
            time.sleep(0.1)
            
            # Choose turn direction based on side distances
            if left_dist > right_dist:
                self.start_turning('LEFT')
            else:
                self.start_turning('RIGHT')
                
        elif front_dist < self.safety_distance:
            # Slow down and prepare to turn
            self.get_logger().info(f"Obstacle ahead! Distance: {front_dist:.2f}m")
            if self.state == 'FORWARD':
                # Choose turn direction based on side distances
                if left_dist > right_dist:
                    self.start_turning('LEFT')
                else:
                    self.start_turning('RIGHT')

def main(args=None):
    rclpy.init(args=args)
    robot = MobileRobotController()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Stopping motors")
        robot.motors.stop()
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
