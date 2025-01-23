# motor_controller.py
from gpiozero import PWMOutputDevice
import time

class MotorController:
    def __init__(self, left_pin, right_pin, forward_min, forward_max, 
                 reverse_min, reverse_max, neutral):
        # PWM parameters from ROS
        self.forward_min = forward_min
        self.forward_max = forward_max
        self.reverse_min = reverse_min  # Closest to neutral
        self.reverse_max = reverse_max  # Furthest from neutral
        self.neutral = neutral
        
        # PWM devices
        self.left_motor = PWMOutputDevice(left_pin, frequency=50, initial_value=neutral)
        self.right_motor = PWMOutputDevice(right_pin, frequency=50, initial_value=neutral)
        time.sleep(0.2)  # Allow PWM initialization

    def set_speeds(self, left_pwm: float, right_pwm: float):
        """Set motors with proper PWM clamping"""
        self.left_motor.value = self._clamp_pwm(left_pwm)
        self.right_motor.value = self._clamp_pwm(right_pwm)

    def _clamp_pwm(self, value: float) -> float:
        """Ensure PWM stays within safe operational range with smooth transitions"""
        if abs(value - self.neutral) < 0.05:  # Dead zone for stability
            return self.neutral
        
        if value > self.neutral:
            # Scale the forward range
            normalized = (value - self.neutral) / (1.0 - self.neutral)
            scaled = self.forward_min + normalized * (self.forward_max - self.forward_min)
            return min(max(scaled, self.forward_min), self.forward_max)
        else:
            # Scale the reverse range
            normalized = (self.neutral - value) / self.neutral
            scaled = self.reverse_min + normalized * (self.reverse_max - self.reverse_min)
            return min(max(scaled, self.reverse_max), self.reverse_min)

    def __del__(self):
        """Cleanup resources"""
        self.left_motor.close()
        self.right_motor.close()