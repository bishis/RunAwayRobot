#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Parameters
        self.declare_parameter('left_pin', 12)
        self.declare_parameter('right_pin', 13)
        self.declare_parameter('safety_timeout', 0.5)
        
        # PWM calibration parameters
        self.declare_parameter('forward_min_duty', 0.09)
        self.declare_parameter('forward_max_duty', 0.10)
        self.declare_parameter('reverse_min_duty', 0.05)
        self.declare_parameter('reverse_max_duty', 0.045)
        self.declare_parameter('neutral_duty', 0.075)
        
        # Get parameters
        params = {param.name: param.value for param in self._parameters.values()}
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=params['left_pin'],
            right_pin=params['right_pin'],
            forward_min=params['forward_min_duty'],
            forward_max=params['forward_max_duty'],
            reverse_min=params['reverse_min_duty'],
            reverse_max=params['reverse_max_duty'],
            neutral=params['neutral_duty']
        )
        
        # Subscribe to wheel speeds
        self.wheel_speeds_sub = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        # Safety system
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        self.get_logger().info('Hardware controller initialized')

    def safety_check(self):
        """Emergency stop if no recent commands"""
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9 * self.get_parameter('safety_timeout').value:
            self.motors.set_speeds(self.get_parameter('neutral_duty').value, 
                                 self.get_parameter('neutral_duty').value)
            self.get_logger().warn('Safety stop activated', once=True)

    def wheel_speeds_callback(self, msg: Twist):
        """Set motor speeds directly from wheel_speeds topic"""
        self.motors.set_speeds(msg.linear.x, msg.angular.z)  # Using linear.x for left, angular.z for right
        self.last_cmd_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()