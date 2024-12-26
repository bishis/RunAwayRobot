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
        
        # Control parameters
        self.max_linear_speed = 1.0
        self.max_angular_speed = 2.0
        self.wheel_separation = 0.2  # meters
        
        # Ensure motors are stopped at start
        self.stop()
        time.sleep(0.1)
        
    def set_speeds(self, left_speed, right_speed):
        """Set motor speeds between -1 and 1."""
        # Clamp values
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)
        
        try:
            self.motor_left.value = left_speed
            self.motor_right.value = right_speed
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
            
    def move(self, linear_speed, angular_speed):
        """Move robot with given linear and angular speeds."""
        # Convert to wheel speeds
        left_speed = linear_speed - (angular_speed * self.wheel_separation / 2.0)
        right_speed = linear_speed + (angular_speed * self.wheel_separation / 2.0)
        
        # Scale to motor range (-1 to 1)
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
            
        self.set_speeds(left_speed, right_speed)
        
    def move_to_pose(self, current_x, current_y, current_yaw, target_x, target_y):
        """Move towards target pose using proportional control."""
        # Calculate distance and angle to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - current_yaw
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        # Proportional control
        linear_speed = min(distance * 2.0, self.max_linear_speed)  # Kp = 2.0
        angular_speed = min(angle_diff * 3.0, self.max_angular_speed)  # Kp = 3.0
        
        # Reduce linear speed when turning sharply
        if abs(angle_diff) > math.pi/4:
            linear_speed *= 0.5
            
        self.move(linear_speed, angular_speed)
        
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