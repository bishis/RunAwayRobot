from gpiozero import PWMOutputDevice
import time
import math

class MotorController:
    """Handles low-level motor control operations using PWMOutputDevice."""
    
    def __init__(self, left_pin: int, right_pin: int):
        """Initialize motor controller with PWM"""
        self.left_motor = PWMOutputDevice(left_pin, frequency=50, initial_value=0.075)
        self.right_motor = PWMOutputDevice(right_pin, frequency=50, initial_value=0.075)
        
        # Store neutral position
        self.neutral = 0.075
        
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
        self.left_motor.value = speed
        self.right_motor.value = speed
        
    def backward(self):
        """Move backward."""
        self.left_motor.value = -self.HALF_SPEED
        self.right_motor.value = -self.HALF_SPEED
        
    def turn_left(self):
        """Turn left in place."""
        self.left_motor.value = -self.TURN_SPEED
        self.right_motor.value = self.TURN_SPEED
        
    def turn_right(self):
        """Turn right in place."""
        self.left_motor.value = self.TURN_SPEED
        self.right_motor.value = -self.TURN_SPEED
        
    def stop(self):
        """Stop motors (neutral position)"""
        self.left_motor.value = self.neutral
        self.right_motor.value = self.neutral
        time.sleep(0.1)
        
    def set_speeds(self, left_pwm: float, right_pwm: float):
        """Set motor speeds using PWM values"""
        # Ensure values are in valid range
        left_pwm = max(0.05, min(0.1, left_pwm))
        right_pwm = max(0.05, min(0.1, right_pwm))
        
        # Apply PWM values
        self.left_motor.value = left_pwm
        self.right_motor.value = right_pwm
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop()
        self.left_motor.close()
        self.right_motor.close() 