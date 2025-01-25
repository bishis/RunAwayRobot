#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Parameters
        self.declare_parameter('speed_channel_pin', 12)  # Changed from left_motor_pin
        self.declare_parameter('turn_channel_pin', 13)   # Changed from right_motor_pin
        self.declare_parameter('max_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # Get parameters
        speed_pin = self.get_parameter('speed_channel_pin').value
        turn_pin = self.get_parameter('turn_channel_pin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Initialize motor controller with servo control
        self.motor_controller = MotorController(
            speed_pin=speed_pin,
            turn_pin=turn_pin,
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
        
        # Calculate linear and angular components
        linear = (left_speed + right_speed) / 2.0    # Average for forward/reverse
        angular = (right_speed - left_speed) / 2.0   # Difference for turning
        
        # Set motor speeds using channels
        self.motor_controller.set_speeds(linear, angular)
        
        # Debug logging
        self.get_logger().info(
            f'Motor Speeds:\n'
            f'  Linear: {linear:.3f}\n'
            f'  Angular: {angular:.3f}'
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