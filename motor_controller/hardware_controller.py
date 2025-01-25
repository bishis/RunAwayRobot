#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Parameters
        self.declare_parameter('speed_channel_pin', 12)
        self.declare_parameter('turn_channel_pin', 13)
        self.declare_parameter('max_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # Get parameters
        speed_pin = self.get_parameter('speed_channel_pin').value
        turn_pin = self.get_parameter('turn_channel_pin').value
        
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
        """Pass servo values directly to motor controller"""
        # Values should already be in -1 to 1 range from navigation controller
        linear = msg.linear.x   # -1 to 1 for reverse/forward
        angular = msg.angular.z  # -1 to 1 for left/right
        
        # Set motor speeds using channels
        self.motor_controller.set_speeds(linear, angular)
        
        # Debug logging
        self.get_logger().info(
            f'Motor Speeds:\n'
            f'  Speed: {linear:4.1f} {"(FWD)" if linear > 0 else "(REV)" if linear < 0 else "(STOP)"}\n'
            f'  Turn: {angular:4.1f} {"(RIGHT)" if angular > 0 else "(LEFT)" if angular < 0 else "(CENTER)"}'
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