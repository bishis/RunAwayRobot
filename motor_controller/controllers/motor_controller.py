from gpiozero import PWMOutputDevice
import time
import math

class MotorController:
    """Handles low-level motor control operations using PWMOutputDevice."""
    
    def __init__(self, left_pin: int, right_pin: int):
        """Initialize motor controller with PWM"""
        # PWM calibration values
        self.NEUTRAL = 0.075  # Neutral/stop position
        self.MIN_FORWARD = 0.09  # Minimum forward duty cycle
        self.MAX_FORWARD = 0.10  # Maximum forward duty cycle
        self.MIN_REVERSE = 0.05  # Minimum reverse duty cycle
        self.MAX_REVERSE = 0.06  # Maximum reverse duty cycle
        
        # Initialize motors with neutral position
        self.left_motor = PWMOutputDevice(left_pin, frequency=50, initial_value=self.NEUTRAL)
        self.right_motor = PWMOutputDevice(right_pin, frequency=50, initial_value=self.NEUTRAL)
        
        # Ensure motors are stopped at start
        self.stop()
        time.sleep(0.1)  # Give servos time to reach neutral
        
        # Speed constants
        self.FULL_SPEED = 1.0
        self.HALF_SPEED = 0.8
        self.TURN_SPEED = 1.0
        self.STOP = 0.0
        
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
        self.left_motor.value = self.NEUTRAL
        self.right_motor.value = self.NEUTRAL
        time.sleep(0.1)  # Give servos time to reach neutral
        
    def set_speeds(self, left_pwm: float, right_pwm: float):
        """Set motor speeds using PWM values"""
        # If speeds are very close to zero, use exact neutral
        if abs(left_pwm - self.NEUTRAL) < 0.001:
            left_pwm = self.NEUTRAL
        if abs(right_pwm - self.NEUTRAL) < 0.001:
            right_pwm = self.NEUTRAL
            
        # Ensure values are in valid range and apply deadband
        left_pwm = self._apply_deadband(left_pwm)
        right_pwm = self._apply_deadband(right_pwm)
        
        # Apply PWM values
        self.left_motor.value = left_pwm
        self.right_motor.value = right_pwm
        
    def _apply_deadband(self, pwm: float) -> float:
        """Apply deadband and limits to PWM value"""
        if pwm > self.NEUTRAL:  # Forward
            if pwm < self.MIN_FORWARD:
                return self.NEUTRAL
            return min(pwm, self.MAX_FORWARD)
        elif pwm < self.NEUTRAL:  # Reverse
            if pwm > self.MIN_REVERSE:
                return self.NEUTRAL
            return max(pwm, self.MAX_REVERSE)
        return self.NEUTRAL
    
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop()
        self.left_motor.close()
        self.right_motor.close() 