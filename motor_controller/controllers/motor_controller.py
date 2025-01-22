from gpiozero import PWMOutputDevice
import time
import math

class MotorController:
    """Handles low-level motor control operations using PWMOutputDevice."""
    
    def __init__(self, left_pin: int, right_pin: int):
        """Initialize motor controller with PWM"""
        # PWM calibration values - adjusted for servo control
        self.NEUTRAL = 0.075      # Center position (1.5ms)
        self.DEADBAND = 0.001     # Small deadband
        
        # Forward settings (1.6ms - 2.0ms)
        self.MIN_FORWARD = 0.080
        self.MAX_FORWARD = 0.100
        
        # Reverse settings (1.0ms - 1.4ms)
        self.MIN_REVERSE = 0.070
        self.MAX_REVERSE = 0.050
        
        # Turn settings
        self.MIN_TURN = 0.077     # Just above neutral
        self.MAX_TURN = 0.095     # Strong turn but not maximum
        
        # Initialize motors
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
        
        print(f"Initializing motors on pins {left_pin} and {right_pin}")
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
        """Set motor speeds with improved turning control"""
        # Print incoming values
        print(f"Incoming PWM - Left: {left_pwm:.4f}, Right: {right_pwm:.4f}")
        
        # Apply deadband around neutral
        if abs(left_pwm - self.NEUTRAL) < self.DEADBAND:
            left_pwm = self.NEUTRAL
        if abs(right_pwm - self.NEUTRAL) < self.DEADBAND:
            right_pwm = self.NEUTRAL
        
        # Detect turning
        is_turning = abs(left_pwm - right_pwm) > self.DEADBAND
        
        if is_turning:
            print("Turning detected")
            # Enhance turn by increasing difference between wheels
            if abs(left_pwm) > abs(right_pwm):
                left_pwm = self._adjust_turn_speed(left_pwm)
                right_pwm = self._adjust_turn_speed(-right_pwm)
            else:
                left_pwm = self._adjust_turn_speed(-left_pwm)
                right_pwm = self._adjust_turn_speed(right_pwm)
        
        # Apply final limits
        left_pwm = self._apply_deadband(left_pwm)
        right_pwm = self._apply_deadband(right_pwm)
        
        # Debug output
        print(f"Final PWM - Left: {left_pwm:.4f}, Right: {right_pwm:.4f}")
        
        # Set motor values
        self.left_motor.value = left_pwm
        self.right_motor.value = right_pwm
    
    def _adjust_turn_speed(self, speed: float) -> float:
        """Adjust speed for turning"""
        if abs(speed) < self.MIN_TURN:
            return self.NEUTRAL
        
        if speed > 0:
            return min(speed, self.MAX_TURN)
        else:
            return max(speed, -self.MAX_TURN)
    
    def _apply_deadband(self, pwm: float) -> float:
        """Apply deadband and limits with hysteresis"""
        if abs(pwm - self.NEUTRAL) < self.DEADBAND:
            return self.NEUTRAL
            
        if pwm > self.NEUTRAL:  # Forward
            if pwm < self.MIN_FORWARD:
                return self.NEUTRAL
            return min(pwm, self.MAX_FORWARD)
        else:  # Reverse
            if pwm > self.MIN_REVERSE:
                return self.NEUTRAL
            return max(pwm, self.MAX_REVERSE)
    
    def stop(self):
        """Stop motors with controlled deceleration"""
        # Gradually move to neutral
        current_left = self.left_motor.value
        current_right = self.right_motor.value
        
        steps = 5
        for i in range(steps):
            left_step = current_left + (self.NEUTRAL - current_left) * (i + 1) / steps
            right_step = current_right + (self.NEUTRAL - current_right) * (i + 1) / steps
            self.left_motor.value = left_step
            self.right_motor.value = right_step
            time.sleep(0.02)
        
        # Final force to neutral
        self.force_stop()
    
    def __del__(self):
        """Cleanup with controlled stop"""
        try:
            self.stop()
            self.left_motor.close()
            self.right_motor.close()
        except:
            pass 