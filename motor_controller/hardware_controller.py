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
        self.declare_parameter('linear_threshold', 0.05)
        self.declare_parameter('angular_threshold', 0.1)
        self.declare_parameter('safety_timeout', 0.5)
        
        # PWM calibration parameters
        self.declare_parameter('forward_min_duty', 0.09)
        self.declare_parameter('forward_max_duty', 0.10)
        self.declare_parameter('reverse_min_duty', 0.05)
        self.declare_parameter('neutral_duty', 0.075)
        self.declare_parameter('speed_exponent', 2.0)
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.get_parameter('left_pin').value,
            right_pin=self.get_parameter('right_pin').value
        )
        
        # Load parameters
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        
        # PWM calibration values
        self.forward_min = self.get_parameter('forward_min_duty').value
        self.forward_max = self.get_parameter('forward_max_duty').value
        self.reverse_min = self.get_parameter('reverse_min_duty').value
        self.neutral = self.get_parameter('neutral_duty').value
        self.exponent = self.get_parameter('speed_exponent').value
        
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
        
        self.get_logger().info('Calibrated Motor Controller Ready')
    
    def safety_check(self):
        """Stop motors if no commands received recently"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if dt > self.safety_timeout:
            self.motors.set_speeds(0, 0)
            self.get_logger().warn('Safety stop triggered', once=True)
    
    def map_speed(self, speed_percent: float) -> float:
        """Map speed (-100% to +100%) to PWM duty cycle"""
        if speed_percent >= 0:
            # Forward: Apply exponential scaling
            normalized = (speed_percent / 100.0) ** self.exponent
            return self.forward_min + normalized * (self.forward_max - self.forward_min)
        else:
            # Reverse: Linear mapping
            return self.reverse_min + (speed_percent / 100.0) * (self.neutral - self.reverse_min)
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to calibrated motor commands"""
        # Prioritize rotation for better control
        if abs(msg.angular.z) > self.angular_threshold:
            # Convert angular velocity to differential drive
            left_percent = -100.0 * (msg.angular.z / 1.0)  # Scale to ±100%
            right_percent = 100.0 * (msg.angular.z / 1.0)
        else:
            # Convert linear velocity to forward/reverse
            if abs(msg.linear.x) < self.linear_threshold:
                left_percent = right_percent = 0.0
            else:
                speed_percent = 100.0 * (msg.linear.x / 0.1)  # Scale to ±100%
                left_percent = right_percent = speed_percent
        
        # Map speeds and apply to motors
        left_pwm = self.map_speed(left_percent)
        right_pwm = self.map_speed(right_percent)
        self.motors.set_speeds(left_pwm, right_pwm)
        
        # Update safety timer
        self.last_cmd_time = self.get_clock().now()
        
        # Debug logging
        self.get_logger().debug(
            f'CMD_VEL: lin={msg.linear.x:.2f} ang={msg.angular.z:.2f} -> '
            f'PWM: L={left_pwm:.3f} R={right_pwm:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()