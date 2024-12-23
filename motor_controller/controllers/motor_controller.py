from gpiozero import Servo

class MotorController:
    """Handles low-level motor control operations."""
    
    def __init__(self, left_pin=18, right_pin=12):
        self.motor_left = Servo(left_pin)
        self.motor_right = Servo(right_pin)
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
    def set_speeds(self, left_speed, right_speed):
        """Set motor speeds between -1 and 1."""
        self.motor_left.value = max(min(left_speed, 1), -1)
        self.motor_right.value = max(min(right_speed, 1), -1)
        self.left_wheel_vel = left_speed
        self.right_wheel_vel = right_speed
        
    def stop(self):
        """Stop both motors."""
        self.set_speeds(0.0, 0.0) 