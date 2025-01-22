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
        
        # Robot physical parameters
        self.track_width = 0.25  # Distance between wheels in meters
        self.max_linear_speed = 0.1  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        # Adjust thresholds for better turning
        self.linear_threshold = 0.01  # Reduced to allow slower movements
        self.angular_threshold = 0.05  # Reduced for more responsive turning
        
        # Subscribe to navigation commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscribed to cmd_vel')
        # Safety timer
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Calibrated Motor Controller Ready')
    
    def safety_check(self):
        """Stop motors if no commands received recently"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if dt > self.safety_timeout:
            # Use exact neutral value for stop
            self.motors.set_speeds(self.neutral, self.neutral)
            self.get_logger().warn('Safety stop triggered', once=True)
    
    def map_speed(self, speed_percent: float) -> float:
        """Map speed (-100% to +100%) to PWM duty cycle"""
        # More precise neutral threshold
        if abs(speed_percent) < 0.5:  # Reduced threshold to 0.5%
            return self.neutral
            
        if speed_percent > 0:
            # Forward: Apply exponential scaling
            normalized = (speed_percent / 100.0) ** self.exponent
            return self.forward_min + normalized * (self.forward_max - self.forward_min)
        else:
            # Reverse: Linear mapping with adjusted neutral
            normalized = abs(speed_percent) / 100.0
            return self.reverse_min + normalized * (self.neutral - self.reverse_min)
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to differential drive commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate differential drive
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Convert to percentages (-100 to 100)
        left_percent = (left_speed / self.max_linear_speed) * 100.0
        right_percent = (right_speed / self.max_linear_speed) * 100.0
        
        # Debug raw values
        self.get_logger().info(
            f'Raw speeds - Left: {left_speed:.3f} m/s, Right: {right_speed:.3f} m/s'
        )
        
        # Prioritize turning at low speeds
        if abs(angular_z) > self.angular_threshold:
            # Enhance turning by increasing difference between wheels
            turn_boost = 1.5
            if abs(left_percent) < abs(right_percent):
                left_percent *= turn_boost
            else:
                right_percent *= turn_boost
        
        # Clamp values
        left_percent = max(-100.0, min(100.0, left_percent))
        right_percent = max(-100.0, min(100.0, right_percent))
        
        self.get_logger().info(
            f'CMD_VEL: lin={linear_x:.3f} ang={angular_z:.3f} -> '
            f'Speeds: L={left_percent:.1f}% R={right_percent:.1f}%'
        )
        
        # Map speeds and apply to motors
        left_pwm = self.map_speed(left_percent)
        right_pwm = self.map_speed(right_percent)
        self.motors.set_speeds(left_pwm, right_pwm)
        
        # Update safety timer
        self.last_cmd_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()