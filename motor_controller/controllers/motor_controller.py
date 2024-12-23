from gpiozero import Servo

class MotorController:
    """Handles low-level motor control operations using Servo controllers."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Initialize Servo objects for motors
        self.motor_left = Servo(left_pin)
        self.motor_right = Servo(right_pin)
        
        # Initialize velocities
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
    def set_speeds(self, left_speed, right_speed):
        """
        Set motor speeds between -1 and 1.
        Positive values for forward, negative for reverse.
        """
        # Clamp values between -1 and 1
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)
        
        # Set motor values
        self.motor_left.value = left_speed
        self.motor_right.value = right_speed
        
        # Store velocities
        self.left_wheel_vel = left_speed
        self.right_wheel_vel = right_speed
        
    def stop(self):
        """Stop both motors."""
        self.motor_left.value = 0
        self.motor_right.value = 0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 