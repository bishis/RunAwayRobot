#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

class MotorController:
    def __init__(self, speed_pin, turn_pin, neutral=0.0):
        # Neutral PWM (1.5ms), forward (2.0ms), and reverse (1.0ms)
        self.neutral = neutral
        self.min_pulse_width = 1.0 / 1000  # 1ms full reverse/left
        self.max_pulse_width = 2.0 / 1000  # 2ms full forward/right

        # Use pigpio for better PWM accuracy
        factory = PiGPIOFactory()

        # Initialize servo objects for the Sabertooth channels
        self.speed_channel = Servo(
            pin=speed_pin,  # Controls forward/reverse
            pin_factory=factory,
            min_pulse_width=self.min_pulse_width,
            max_pulse_width=self.max_pulse_width,
            frame_width=20 / 1000  # Standard 20ms frame
        )

        self.turn_channel = Servo(
            pin=turn_pin,  # Controls left/right turning
            pin_factory=factory,
            min_pulse_width=self.min_pulse_width,
            max_pulse_width=self.max_pulse_width,
            frame_width=20 / 1000
        )

        # Set channels to neutral (stop) on initialization
        self.speed_channel.value = neutral
        self.turn_channel.value = neutral
        time.sleep(0.2)

    def set_speeds(self, linear: float, angular: float):
        """
        Update motor speeds based on linear and angular velocity commands.
        - linear: Forward/backward speed (-1.0 to 1.0)
        - angular: Left/right turning speed (-1.0 to 1.0)
        """
        # Clamp values to valid range
        linear = max(min(linear, 1.0), -1.0)
        angular = max(min(angular, 1.0), -1.0)

        # Set channel values directly
        self.speed_channel.value = linear   # Forward/backward
        self.turn_channel.value = angular   # Left/right

    def __del__(self):
        """Ensure the motors are stopped on program exit"""
        self.speed_channel.detach()
        self.turn_channel.detach()