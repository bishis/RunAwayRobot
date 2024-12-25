from gpiozero import Servo
import time

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
        
        # Ensure motors are stopped at start
        self.stop()
        time.sleep(0.1)
        
    def set_speeds(self, left_speed, right_speed):
        """Set motor speeds between -1 and 1."""
        # Clamp values
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)
        
        print(f"Setting speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
        
        try:
            # Convert from -1,1 range to 0,1 range for servos
            self.motor_left.value = left_speed
            self.motor_right.value = right_speed
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
        
    def stop(self):
        """Stop both motors."""
        try:
            self.motor_left.value = 0
            self.motor_right.value = 0
            time.sleep(0.1)
        except Exception as e:
            print(f"Error stopping motors: {e}")
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 