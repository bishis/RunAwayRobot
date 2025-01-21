#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from .controllers.motor_controller import MotorController


class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.declare_parameter('left_pin', 18).value,
            right_pin=self.declare_parameter('right_pin', 12).value
        )
        
        # Subscribe to navigation commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Robot parameters
        self.track_width = 0.3  # Distance between tracks in meters
        self.max_linear_speed = 0.1  # Max linear speed from nav2 params
        self.max_angular_speed = 0.5  # Max angular speed from nav2 params
        
        # Motor control parameters
        self.control_period = 0.5  # Control period in seconds (longer for smoother motion)
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.motor_control_timer)  # 10Hz control loop
        
        # Store motor states
        self.left_active_time = 0.0
        self.right_active_time = 0.0
        self.left_direction = 0
        self.right_direction = 0
        self.cycle_start_time = self.get_clock().now()
        
        # Obstacle detection parameters
        self.min_obstacle_distance = 0.3  # Meters
        self.last_obstacle_warning = self.get_clock().now()
        self.warning_interval = 1.0  # Seconds between warnings
        
        self.get_logger().info('Hardware Controller initialized')
    
    def motor_control_timer(self):
        """Timer callback for smooth motor control"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        # Stop if no recent commands
        if dt > 0.5:
            self.motors.set_speeds(0, 0)
            return
        
        # Calculate time in current control period
        cycle_time = (now - self.cycle_start_time).nanoseconds / 1e9
        
        # Start new control period if needed
        if cycle_time >= self.control_period:
            self.cycle_start_time = now
            cycle_time = 0.0
        
        # Determine motor states based on duty cycle
        left_on = cycle_time < self.left_active_time
        right_on = cycle_time < self.right_active_time
        
        # Set motor speeds
        left_speed = self.left_direction if left_on else 0
        right_speed = self.right_direction if right_on else 0
        
        self.motors.set_speeds(left_speed, right_speed)
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan data and check for obstacles"""
        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if ranges:
            min_distance = min(ranges)
            
            now = self.get_clock().now()
            if (min_distance < self.min_obstacle_distance and 
                (now - self.last_obstacle_warning).nanoseconds / 1e9 >= self.warning_interval):
                self.get_logger().warn(f'Obstacle detected! Distance: {min_distance:.2f}m')
                self.last_obstacle_warning = now
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel into motor control parameters"""
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Normalize speeds
        left_norm = self.normalize_speed(left_speed)
        right_norm = self.normalize_speed(right_speed)
        
        # Convert normalized speeds to duty cycle and direction
        self.left_active_time = abs(left_norm) * self.control_period
        self.right_active_time = abs(right_norm) * self.control_period
        self.left_direction = 1 if left_norm > 0 else (-1 if left_norm < 0 else 0)
        self.right_direction = 1 if right_norm > 0 else (-1 if right_norm < 0 else 0)
        
        self.last_cmd_time = self.get_clock().now()
    
    def normalize_speed(self, speed: float) -> float:
        """Normalize speed to -1 to 1 range"""
        max_speed = self.max_linear_speed + (self.max_angular_speed * self.track_width / 2.0)
        normalized = speed / max_speed
        return max(min(normalized, 1.0), -1.0)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
