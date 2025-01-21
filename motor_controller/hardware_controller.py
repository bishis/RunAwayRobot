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
        self.pulse_duration = 0.1  # Duration of each motor pulse in seconds
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(self.pulse_duration, self.motor_control_timer)
        
        # Store current desired speeds
        self.current_left = 0
        self.current_right = 0
        self.desired_left = 0
        self.desired_right = 0
        
        # Obstacle detection parameters
        self.min_obstacle_distance = 0.3  # Meters
        self.last_obstacle_warning = self.get_clock().now()
        self.warning_interval = 1.0  # Seconds between warnings
        
        self.get_logger().info('Hardware Controller initialized')
    
    def motor_control_timer(self):
        """Timer callback to implement pulse-width control"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        # If no command received recently, stop motors
        if dt > 0.5:  # Stop if no commands for 0.5 seconds
            self.motors.set_speeds(0, 0)
            return
            
        # Calculate duty cycle based on desired speed
        if abs(self.desired_left) > 0.3:  # Full speed
            self.current_left = 1 if self.desired_left > 0 else -1
        elif abs(self.desired_left) > 0:  # Pulse for lower speeds
            if self.current_left == 0:
                self.current_left = 1 if self.desired_left > 0 else -1
            else:
                self.current_left = 0
                
        if abs(self.desired_right) > 0.3:  # Full speed
            self.current_right = 1 if self.desired_right > 0 else -1
        elif abs(self.desired_right) > 0:  # Pulse for lower speeds
            if self.current_right == 0:
                self.current_right = 1 if self.desired_right > 0 else -1
            else:
                self.current_right = 0
        
        self.motors.set_speeds(self.current_left, self.current_right)
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan data and check for obstacles"""
        # Find the minimum distance in the scan
        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if ranges:
            min_distance = min(ranges)
            
            # Check if it's time to print a new warning
            now = self.get_clock().now()
            if (min_distance < self.min_obstacle_distance and 
                (now - self.last_obstacle_warning).nanoseconds / 1e9 >= self.warning_interval):
                self.get_logger().warn(f'Obstacle detected! Distance: {min_distance:.2f}m')
                self.last_obstacle_warning = now
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel into left/right motor commands"""
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Normalize speeds
        self.desired_left = self.normalize_speed(left_speed)
        self.desired_right = self.normalize_speed(right_speed)
        
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
