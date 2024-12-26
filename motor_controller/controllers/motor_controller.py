from gpiozero import Servo
import time
import math

class MotorController:
    """Handles low-level motor control operations using Servo controllers."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Initialize Servo objects for motors
        self.motor_left = Servo(
            left_pin,
            min_pulse_width=1.0/1000,
            max_pulse_width=2.0/1000,
            frame_width=20.0/1000
        )
        
        self.motor_right = Servo(
            right_pin,
            min_pulse_width=1.0/1000,
            max_pulse_width=2.0/1000,
            frame_width=20.0/1000
        )
        
        # Movement states
        self.FULL_SPEED = 1.0
        self.STOP = 0.0
        
        # Ensure motors are stopped at start
        self.stop()
        
    def forward(self):
        """Move forward at full speed."""
        self.motor_left.value = self.FULL_SPEED
        self.motor_right.value = self.FULL_SPEED
        
    def backward(self):
        """Move backward at full speed."""
        self.motor_left.value = -self.FULL_SPEED
        self.motor_right.value = -self.FULL_SPEED
        
    def turn_left(self):
        """Turn left in place at full speed."""
        self.motor_left.value = -self.FULL_SPEED
        self.motor_right.value = self.FULL_SPEED
        
    def turn_right(self):
        """Turn right in place at full speed."""
        self.motor_left.value = self.FULL_SPEED
        self.motor_right.value = -self.FULL_SPEED
        
    def stop(self):
        """Stop both motors."""
        self.motor_left.value = self.STOP
        self.motor_right.value = self.STOP
        time.sleep(0.1)  # Short delay to ensure stop
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 