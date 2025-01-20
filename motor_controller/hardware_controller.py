#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
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
        
        # Robot parameters
        self.track_width = 0.3  # Distance between tracks in meters
        self.max_linear_speed = 0.1  # Max linear speed from nav2 params
        self.max_angular_speed = 0.5  # Max angular speed from nav2 params
        
        self.get_logger().info('Hardware Controller initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel into left/right motor commands"""
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Normalize and convert to binary (-1 or 1)
        left_speed = self.normalize_speed(left_speed)
        right_speed = self.normalize_speed(right_speed)
        
        # Convert to binary speeds (-1 or 1)
        left_speed = 1 if left_speed > 0 else (-1 if left_speed < 0 else 0)
        right_speed = 1 if right_speed > 0 else (-1 if right_speed < 0 else 0)
        
        # Send commands to motors
        self.motors.set_speeds(left_speed, right_speed)
        
        # Log commands
        self.get_logger().debug(
            f'Motors: L:{left_speed:.0f} R:{right_speed:.0f} (cmd_vel: lin:{linear_x:.2f} ang:{angular_z:.2f})'
        )
    
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
