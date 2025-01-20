from gpiozero import Servo
import time
import math

class MotorController:
    """Handles low-level motor control operations using Servo controllers."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Initialize Servo objects for motors
        self.motor_left = Servo(
            left_pin
        )
        
        self.motor_right = Servo(
            right_pin
        )
        
        # Speed constants
        self.FULL_SPEED = 1.0
        self.HALF_SPEED = 0.8
        self.TURN_SPEED = 1.0
        self.STOP = 0.0
        
        # Ensure motors are stopped at start
        self.stop()
        
    def forward(self, speed=None):
        """Move forward at specified speed."""
        speed = speed or self.HALF_SPEED
        self.motor_left.value = speed
        self.motor_right.value = speed
        
    def backward(self):
        """Move backward."""
        self.motor_left.value = -self.HALF_SPEED
        self.motor_right.value = -self.HALF_SPEED
        
    def turn_left(self):
        """Turn left in place."""
        self.motor_left.value = -self.TURN_SPEED
        self.motor_right.value = self.TURN_SPEED
        
    def turn_right(self):
        """Turn right in place."""
        self.motor_left.value = self.TURN_SPEED
        self.motor_right.value = -self.TURN_SPEED
        
    def stop(self):
        """Stop both motors."""
        self.motor_left.value = self.STOP
        self.motor_right.value = self.STOP
        time.sleep(0.1)
        
    def set_speeds(self, left_speed, right_speed):
        """Set speeds for both motors."""

        # Apply speeds to motors
        self.motor_left.value = left_speed
        self.motor_right.value = right_speed
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 