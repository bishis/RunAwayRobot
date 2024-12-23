from gpiozero import PWMOutputDevice

class MotorController:
    """Handles low-level motor control operations."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Initialize PWM outputs for motors
        self.motor_left = PWMOutputDevice(left_pin, frequency=1000)
        self.motor_right = PWMOutputDevice(right_pin, frequency=1000)
        
        # Add direction pins
        self.left_dir = PWMOutputDevice(23)  # GPIO 23 for left motor direction
        self.right_dir = PWMOutputDevice(24)  # GPIO 24 for right motor direction
        
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
    def set_speeds(self, left_speed, right_speed):
        """Set motor speeds between -1 and 1."""
        # Handle left motor
        if left_speed >= 0:
            self.left_dir.off()  # Forward
            self.motor_left.value = min(abs(left_speed), 1.0)
        else:
            self.left_dir.on()   # Reverse
            self.motor_left.value = min(abs(left_speed), 1.0)
            
        # Handle right motor
        if right_speed >= 0:
            self.right_dir.off()  # Forward
            self.motor_right.value = min(abs(right_speed), 1.0)
        else:
            self.right_dir.on()   # Reverse
            self.motor_right.value = min(abs(right_speed), 1.0)
            
        self.left_wheel_vel = left_speed
        self.right_wheel_vel = right_speed
        
    def stop(self):
        """Stop both motors."""
        self.motor_left.value = 0
        self.motor_right.value = 0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0 