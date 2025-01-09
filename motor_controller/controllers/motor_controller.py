from gpiozero import DigitalOutputDevice
import time

class MotorController:
    """Handles low-level motor control operations using direct GPIO."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Initialize GPIO pins for motors
        self.motor_left = DigitalOutputDevice(left_pin)
        self.motor_right = DigitalOutputDevice(right_pin)
        
        # Speed constants
        self.FULL_SPEED = 1.0
        self.HALF_SPEED = 0.8
        self.TURN_SPEED = 1.0
        self.STOP = 0.0
        
        # Ensure motors are stopped at start
        self.stop()
        
    def forward(self, speed=None):
        """Move forward at specified speed."""
        self.motor_left.on()
        self.motor_right.on()
        
    def backward(self):
        """Move backward."""
        self.motor_left.off()
        self.motor_right.off()
        
    def turn_left(self):
        """Turn left in place."""
        self.motor_left.off()
        self.motor_right.on()
        
    def turn_right(self):
        """Turn right in place."""
        self.motor_left.on()
        self.motor_right.off()
        
    def stop(self):
        """Stop both motors."""
        self.motor_left.off()
        self.motor_right.off()
        
    def set_speeds(self, left_speed, right_speed):
        """Set speeds for both motors."""
        # Convert speeds to on/off
        self.motor_left.value = 1 if left_speed > 0 else 0
        self.motor_right.value = 1 if right_speed > 0 else 0
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 