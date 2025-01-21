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
        self.control_period = 0.1  # 100ms control period
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(self.control_period, self.motor_control_timer)
        
        # Store current motor states
        self.target_left = 0
        self.target_right = 0
        
        # Movement thresholds
        self.min_speed_threshold = 0.2    # Increased threshold for more decisive movements
        
        # Obstacle detection parameters
        self.min_obstacle_distance = 0.3  # Meters
        self.last_obstacle_warning = self.get_clock().now()
        self.warning_interval = 1.0  # Seconds between warnings
        
        # Add timing diagnostics
        self.last_control_time = self.get_clock().now()
        self.cmd_vel_count = 0
        self.create_timer(1.0, self.print_diagnostics)  # Print stats every second
        
        self.get_logger().info('Hardware Controller initialized')
    
    def motor_control_timer(self):
        """Timer callback for pure binary motor control"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        # Stop if no recent commands
        if dt > 0.5:
            self.target_left = 0
            self.target_right = 0
            self.motors.set_speeds(0, 0)
            return
        
        # Determine primary motion type
        linear_motion = abs(self.target_left + self.target_right) > abs(self.target_left - self.target_right)
        
        # Convert to binary outputs
        if linear_motion:
            # Forward/backward motion takes priority
            if abs(self.target_left + self.target_right) > self.min_speed_threshold:
                direction = 1 if (self.target_left + self.target_right) > 0 else -1
                left_out = right_out = direction
            else:
                left_out = right_out = 0
        else:
            # Rotation
            if abs(self.target_left - self.target_right) > self.min_speed_threshold:
                left_out = 1 if self.target_left > 0 else -1
                right_out = -left_out  # Opposite direction for rotation
            else:
                left_out = right_out = 0
        
        self.motors.set_speeds(left_out, right_out)
        
        # Debug logging
        self.get_logger().debug(
            f'Motion: {"LINEAR" if linear_motion else "ROTATION"} '
            f'Outputs: L={left_out} R={right_out}'
        )
    
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
        """Convert cmd_vel into binary motor commands"""
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Normalize speeds
        self.target_left = self.normalize_speed(left_speed)
        self.target_right = self.normalize_speed(right_speed)
        
        self.last_cmd_time = self.get_clock().now()
    
    def normalize_speed(self, speed: float) -> float:
        """Normalize speed to -1 to 1 range"""
        max_speed = self.max_linear_speed + (self.max_angular_speed * self.track_width / 2.0)
        normalized = speed / max_speed
        return max(min(normalized, 1.0), -1.0)

    def print_diagnostics(self):
        """Print diagnostic information"""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        
        self.get_logger().info(
            f'Diagnostics:\n'
            f'  CMD_VEL frequency: {self.cmd_vel_count} Hz\n'
            f'  Target speeds: L={self.target_left:.2f} R={self.target_right:.2f}\n'
            f'  Time since last cmd: {dt:.3f}s'
        )
        self.cmd_vel_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
