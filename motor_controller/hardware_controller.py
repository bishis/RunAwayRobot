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
        self.declare_parameter('forward_min_duty', 0.08)
        self.declare_parameter('forward_max_duty', 0.15)  # Increased for more power
        self.declare_parameter('reverse_min_duty', 0.07)  # Closest to neutral
        self.declare_parameter('reverse_max_duty', 0.02)  # Furthest from neutral
        self.declare_parameter('neutral_duty', 0.075)
        self.declare_parameter('speed_exponent', 2.0)
        
        # Robot physical parameters
        self.declare_parameter('wheel_separation', 0.24)
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.0)
        
        # Get all parameters
        params = {param.name: param.value for param in self._parameters.values()}
        
        # Initialize motor controller with PWM parameters
        self.motors = MotorController(
            left_pin=params['left_pin'],
            right_pin=params['right_pin'],
            forward_min=params['forward_min_duty'],
            forward_max=params['forward_max_duty'],
            reverse_min=params['reverse_min_duty'],
            reverse_max=params['reverse_max_duty'],
            neutral=params['neutral_duty']
        )
        
        # Load other parameters
        self.wheel_separation = params['wheel_separation']
        self.max_linear_speed = params['max_linear_speed']
        self.max_angular_speed = params['max_angular_speed']
        self.speed_exponent = params['speed_exponent']
        
        # Subscribe to navigation commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscribed to cmd_vel')
        
        # Safety system
        self.last_cmd_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        self.get_logger().info('Motor controller initialized')

    def safety_check(self):
        """Emergency stop if no recent commands"""
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9 * self.get_parameter('safety_timeout').value:
            self.motors.set_speeds(self.get_parameter('neutral_duty').value, 
                                 self.get_parameter('neutral_duty').value)
            self.get_logger().warn('Safety stop activated', once=True)

    def map_speed(self, speed_percent: float) -> float:
        """Convert speed percentage to PWM value with proper direction handling"""
        if abs(speed_percent) < 0.5:
            return self.get_parameter('neutral_duty').value
            
        normalized = abs(speed_percent) / 100.0
        
        if speed_percent > 0:  # Forward
            # Map to forward range
            return self.get_parameter('neutral_duty').value + \
                   normalized * (self.get_parameter('forward_max_duty').value - 
                               self.get_parameter('neutral_duty').value)
        else:  # Reverse
            # Map to reverse range
            return self.get_parameter('neutral_duty').value - \
                   normalized * (self.get_parameter('neutral_duty').value - 
                               self.get_parameter('reverse_max_duty').value)

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist commands to motor speeds"""
        # Clamp input values
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # Convert to wheel velocities
        left_speed = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert to percentages
        left_percent = (left_speed / self.max_linear_speed) * 100.0
        right_percent = (right_speed / self.max_linear_speed) * 100.0

        # Handle spot turns with smooth ramping
        if abs(angular_z) > 0.1 and abs(linear_x) < 0.01:
            # Calculate turn power based on angular velocity
            turn_power = min(100.0, abs(angular_z) / self.max_angular_speed * 100.0)
            # Apply exponential smoothing for gradual acceleration
            turn_power = pow(turn_power / 100.0, self.speed_exponent) * 100.0
            
            if angular_z > 0:  # CCW turn
                left_percent = -turn_power
                right_percent = turn_power
            else:  # CW turn
                left_percent = turn_power
                right_percent = -turn_power

        # Convert to PWM values
        left_pwm = self.map_speed(left_percent)
        right_pwm = self.map_speed(right_percent)

        # Send commands to motors
        self.motors.set_speeds(left_pwm, right_pwm)
        self.last_cmd_time = self.get_clock().now()
        
        # Debug logging
        self.get_logger().debug(
            f'Speeds: L={left_percent:.1f}% R={right_percent:.1f}% | '
            f'PWM: L={left_pwm:.4f} R={right_pwm:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()