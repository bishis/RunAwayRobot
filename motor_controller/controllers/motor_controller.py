from gpiozero import PWMOutputDevice
import time
import math

class MotorController:
    """Handles low-level motor control operations using PWMOutputDevice."""
    
    def __init__(self, left_pin: int, right_pin: int):
        """Initialize motor controller with PWM"""
        # PWM calibration values - adjusted for both forward and reverse turning
        self.NEUTRAL = 0.0725      # Neutral position
        self.DEADBAND = 0.001      # Minimal deadband
        
        # Forward ranges (0.0725 -> 0.100)
        self.MIN_FORWARD = 0.090   # 90% minimum forward
        self.MAX_FORWARD = 0.100   # 100% maximum forward
        
        # Reverse ranges (0.0725 -> 0.045)
        self.MIN_REVERSE = 0.055   # Stronger reverse
        self.MAX_REVERSE = 0.045   # Maximum reverse power
        
        # Initialize motors at exact neutral
        self.left_motor = PWMOutputDevice(
            left_pin, 
            frequency=50,
            initial_value=self.NEUTRAL
        )
        self.right_motor = PWMOutputDevice(
            right_pin,
            frequency=50,
            initial_value=self.NEUTRAL
        )
        
        # Force stop at initialization
        self.force_stop()
        time.sleep(0.2)
        
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
        
    def force_stop(self):
        """Force motors to exact neutral position"""
        self.left_motor.value = self.NEUTRAL
        self.right_motor.value = self.NEUTRAL
        time.sleep(0.1)
    
    def set_speeds(self, left_pwm: float, right_pwm: float):
        """Set motor speeds using PWM values with synchronized application"""
        # Apply deadband around neutral
        if abs(left_pwm - self.NEUTRAL) < self.DEADBAND:
            left_pwm = self.NEUTRAL
        if abs(right_pwm - self.NEUTRAL) < self.DEADBAND:
            right_pwm = self.NEUTRAL
        
        # Apply limits and deadband
        left_pwm = self._apply_deadband(left_pwm)
        right_pwm = self._apply_deadband(right_pwm)
        
        # Debug output for troubleshooting
        print(f"PWM values - Left: {left_pwm:.4f}, Right: {right_pwm:.4f}")
        
        # Set both motor values simultaneously
        self.left_motor.value = left_pwm
        self.right_motor.value = right_pwm
    
    def _apply_deadband(self, pwm: float) -> float:
        """Apply deadband and limits with aggressive power for both directions"""
        if abs(pwm - self.NEUTRAL) < self.DEADBAND:
            return self.NEUTRAL
            
        if pwm > self.NEUTRAL:  # Forward
            # Always use high power when moving forward
            if pwm > self.NEUTRAL + self.DEADBAND:
                return max(self.MIN_FORWARD, min(pwm, self.MAX_FORWARD))
            return self.NEUTRAL
        else:  # Reverse
            # Always use high power when moving reverse
            if pwm < self.NEUTRAL - self.DEADBAND:
                # Note: For reverse, MIN is closer to neutral than MAX
                return min(self.MIN_REVERSE, max(pwm, self.MAX_REVERSE))
            return self.NEUTRAL
    
    def stop(self):
        """Stop motors with synchronized stop"""
        # Set both motors to neutral simultaneously
        self.left_motor.value = self.NEUTRAL
        self.right_motor.value = self.NEUTRAL
        time.sleep(0.1)  # Short delay to ensure stop
    
    def __del__(self):
        """Cleanup with controlled stop"""
        try:
            self.stop()
            self.left_motor.close()
            self.right_motor.close()
        except:
            pass 