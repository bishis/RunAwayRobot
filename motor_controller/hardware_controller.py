#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.track_width = 0.3  # Distance between wheels in meters
        
        # Safety timeout
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.safety_timer_callback)  # 10Hz safety check
        
        self.get_logger().info('Hardware Controller initialized')
    
    def safety_timer_callback(self):
        """Stop motors if no commands received recently"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if dt > 0.5:  # Stop after 0.5 seconds without commands
            self.motors.set_speeds(0, 0)
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel into left/right motor commands"""
        # Extract velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Convert to binary commands (-1, 0, 1)
        # left_speed = self.convert_to_binary(left_speed)
        # right_speed = self.convert_to_binary(right_speed)
        
        # Send commands to motors
        self.motors.set_speeds(left_speed, right_speed)
        
        # Update safety timer
        self.last_cmd_time = self.get_clock().now()
        
        # Debug logging
        self.get_logger().debug(
            f'CMD_VEL: linear={linear_x:.2f} angular={angular_z:.2f} -> '
            f'motors: L={left_cmd} R={right_cmd}'
        )
    
    def convert_to_binary(self, speed: float) -> int:
        """Convert continuous speed to binary command (-1, 0, 1)"""
        threshold = 0.01  # Minimum speed threshold
        
        if abs(speed) < threshold:
            return 0
        return 1 if speed > 0 else -1


def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
