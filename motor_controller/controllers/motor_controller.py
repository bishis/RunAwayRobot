from gpiozero import Servo
import time

class MotorController:
    """Handles low-level motor control operations using Servo controllers."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Initialize Servo objects for motors with specific pulse width ranges
        # Adjust these min_pulse_width and max_pulse_width values based on your servos
        self.motor_left = Servo(
            left_pin,
            min_pulse_width=1.0/1000,  # 1ms
            max_pulse_width=2.0/1000,  # 2ms
            frame_width=20.0/1000      # 20ms frame
        )
        
        self.motor_right = Servo(
            right_pin,
            min_pulse_width=1.0/1000,  # 1ms
            max_pulse_width=2.0/1000,  # 2ms
            frame_width=20.0/1000      # 20ms frame
        )
        
        # Initialize velocities
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        # Ensure motors are stopped at start
        self.stop()
        time.sleep(0.1)  # Short delay to ensure motors initialize properly
        
    def set_speeds(self, left_speed, right_speed):
        """
        Set motor speeds between -1 and 1.
        Positive values for forward, negative for reverse.
        """
        # Debug output
        print(f"Setting speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}")
        
        # Clamp values between -1 and 1
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)
        
        try:
            # Set motor values with error checking
            self.motor_left.value = left_speed
            self.motor_right.value = right_speed
            
            # Store velocities
            self.left_wheel_vel = left_speed
            self.right_wheel_vel = right_speed
            
            # Debug confirmation
            print(f"Motor values set - Left: {self.motor_left.value}, Right: {self.motor_right.value}")
            
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
        
    def stop(self):
        """Stop both motors."""
        print("Stopping motors")
        try:
            self.motor_left.value = 0
            self.motor_right.value = 0
            self.left_wheel_vel = 0.0
            self.right_wheel_vel = 0.0
            time.sleep(0.1)  # Short delay to ensure stop command is processed
        except Exception as e:
            print(f"Error stopping motors: {e}")
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 