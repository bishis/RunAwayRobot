#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

class MotorController:
    def __init__(self, left_pin, right_pin, neutral=0.0):
        # Neutral PWM (1.5ms), forward (2.0ms), and reverse (1.0ms)
        self.neutral = neutral
        self.min_pulse_width = 1.0 / 1000  # 1ms full reverse
        self.max_pulse_width = 2.0 / 1000  # 2ms full forward

        # Use pigpio for better PWM accuracy
        factory = PiGPIOFactory()

        # Initialize servo objects for the Sabertooth motor driver
        self.left_motor = Servo(
            pin=left_pin,
            pin_factory=factory,
            min_pulse_width=self.min_pulse_width,
            max_pulse_width=self.max_pulse_width,
            frame_width=20 / 1000  # Standard 20ms frame
        )

        self.right_motor = Servo(
            pin=right_pin,
            pin_factory=factory,
            min_pulse_width=self.min_pulse_width,
            max_pulse_width=self.max_pulse_width,
            frame_width=20 / 1000
        )

        # Set motors to neutral (stop) on initialization
        self.left_motor.value = neutral
        self.right_motor.value = neutral
        time.sleep(0.2)

    def set_speeds(self, linear: float, angular: float):
        """
        Update motor speeds based on linear and angular velocity commands.
        - linear: Forward/backward speed (-1.0 to 1.0)
        - angular: Left/right turning speed (-1.0 to 1.0)
        """
        # Combine linear and angular velocities for differential drive
        left_speed = linear + angular  # Left motor forward for positive angular
        right_speed = linear - angular  # Right motor forward for negative angular

        # Clamp speeds to valid range
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)

        # Set motor speeds
        self.left_motor.value = self._pwm_to_servo_value(left_speed)
        self.right_motor.value = self._pwm_to_servo_value(right_speed)

    def _pwm_to_servo_value(self, pwm: float) -> float:
        """Convert linear PWM value (-1.0 to 1.0) to Servo value"""
        # Neutral is 0, forward is positive, reverse is negative
        return max(min(pwm, 1.0), -1.0)

    def __del__(self):
        """Ensure the motors are stopped on program exit"""
        self.left_motor.detach()
        self.right_motor.detach()