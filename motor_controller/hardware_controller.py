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
        
        # Robot physical parameters
        self.declare_parameter('wheel_separation', 0.24)
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.0)
        
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
        
        # Load physical parameters
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
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
            self.motors.set_speeds(self.neutral, self.neutral)
            self.get_logger().warn('Safety stop triggered', once=True)
    
    def map_speed(self, speed_percent: float) -> float:
        """Corrected speed mapping with proper reverse handling"""
        if abs(speed_percent) < 0.5:
            return self.neutral
            
        if speed_percent > 0:
            normalized = (speed_percent / 100.0) ** self.exponent
            return self.forward_min + normalized * (self.forward_max - self.forward_min)
        else:
            normalized = abs(speed_percent) / 100.0
            return self.neutral - normalized * (self.neutral - self.reverse_min)
    
    def cmd_vel_callback(self, msg: Twist):
        """Improved motor command handling"""
        # Clamp velocities
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # Reduced angular scaling for smoother turns
        angular_z *= 1.0  # Reduced from 1.5 for more controlled turning

        # Differential drive calculation
        left_speed = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert to percentages
        left_percent = (left_speed / self.max_linear_speed) * 100.0
        right_percent = (right_speed / self.max_linear_speed) * 100.0

        # Special handling for spot turns
        if abs(angular_z) > 0.1 and abs(linear_x) < 0.01:  # Detect spot turn
            turn_power = 80.0  # Use 80% power for turns
            if angular_z > 0:  # Turning left
                left_percent = -turn_power
                right_percent = turn_power
            else:  # Turning right
                left_percent = turn_power
                right_percent = -turn_power
        else:
            # Normal movement - ensure minimum power when moving
            MIN_POWER = 30.0
            if abs(left_percent) > 1.0:  # Only apply if movement is intended
                left_sign = -1 if left_percent < 0 else 1
                left_percent = left_sign * max(abs(left_percent), MIN_POWER)
            if abs(right_percent) > 1.0:  # Only apply if movement is intended
                right_sign = -1 if right_percent < 0 else 1
                right_percent = right_sign * max(abs(right_percent), MIN_POWER)

        # Debug logging
        self.get_logger().info(
            f'CMD_VEL: linear={linear_x:.3f} angular={angular_z:.3f}\n'
            f'Wheel speeds: left={left_speed:.3f} right={right_speed:.3f}\n'
            f'Percentages: left={left_percent:.1f}% right={right_percent:.1f}%'
        )
        
        # Map speeds and apply
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