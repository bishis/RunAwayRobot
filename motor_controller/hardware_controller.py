#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Parameters
        self.declare_parameter('left_motor_pin', 17)
        self.declare_parameter('right_motor_pin', 18)
        self.declare_parameter('max_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # Get parameters
        left_pin = self.get_parameter('left_motor_pin').value
        right_pin = self.get_parameter('right_motor_pin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Initialize motor controller with servo control
        self.motor_controller = MotorController(
            left_pin=left_pin,
            right_pin=right_pin,
            neutral=0.0  # Servo neutral position
        )
        
        # Create subscription
        self.subscription = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        self.get_logger().info('Hardware controller initialized')

    def wheel_speeds_callback(self, msg: Twist):
        """Convert wheel speeds to normalized motor commands"""
        # Convert PWM values to normalized speeds (-1 to 1)
        left_speed = (msg.linear.x - 0.075) * 40  # Convert from PWM to -1 to 1
        right_speed = (msg.angular.z - 0.075) * 40
        
        # Set motor speeds
        self.motor_controller.set_speeds(
            linear=(left_speed + right_speed) / 2.0,  # Average for linear motion
            angular=(right_speed - left_speed) / 2.0   # Difference for turning
        )
        
        # Debug logging
        self.get_logger().info(
            f'Motor Speeds:\n'
            f'  Left: {left_speed:.3f}\n'
            f'  Right: {right_speed:.3f}'
        )

    def destroy_node(self):
        """Clean up on shutdown"""
        if hasattr(self, 'motor_controller'):
            del self.motor_controller
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()