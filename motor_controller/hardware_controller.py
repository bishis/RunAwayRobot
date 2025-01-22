#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Parameters
        self.declare_parameter('left_pin', 18)
        self.declare_parameter('right_pin', 12)
        self.declare_parameter('track_width', 0.2)  # Meters between wheels
        self.declare_parameter('linear_threshold', 0.05)  # Min linear speed to move
        self.declare_parameter('angular_threshold', 0.1)  # Min angular speed to turn
        self.declare_parameter('safety_timeout', 0.5)  # Seconds before auto-stop
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.get_parameter('left_pin').value,
            right_pin=self.get_parameter('right_pin').value
        )
        
        # Load parameters
        self.track_width = self.get_parameter('track_width').value
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        
        # Subscribe to navigation commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timer
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Binary Motor Controller Ready')

    def safety_check(self):
        """Stop motors if no commands received recently"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if dt > self.safety_timeout:
            self.motors.set_speeds(0, 0)
            self.get_logger().warn('Safety stop triggered', once=True)

    def cmd_vel_callback(self, msg: Twist):
        """Convert continuous cmd_vel to binary motor commands"""
        # Prioritize rotation for better obstacle avoidance
        if abs(msg.angular.z) > self.angular_threshold:
            self.handle_rotation(msg.angular.z)
        else:
            self.handle_linear(msg.linear.x)
        
        self.last_cmd_time = self.get_clock().now()

    def handle_linear(self, linear_x: float):
        """Handle forward/backward movement"""
        if abs(linear_x) < self.linear_threshold:
            self.motors.set_speeds(0, 0)
            return
        
        # Binary forward/reverse
        cmd = 1 if linear_x > 0 else -1
        self.motors.set_speeds(cmd, cmd)
        self.get_logger().debug(f'Linear move: {cmd}')

    def handle_rotation(self, angular_z: float):
        """Handle in-place rotations"""
        # Binary left/right turn
        if angular_z > 0:
            self.motors.set_speeds(1, -1)  # Right turn
        else:
            self.motors.set_speeds(-1, 1)  # Left turn
        self.get_logger().debug(f'Rotating: {"right" if angular_z > 0 else "left"}')

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()