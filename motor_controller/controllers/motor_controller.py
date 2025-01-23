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
        """Ensure PWM stays within safe operational range"""
        if value > self.neutral:
            return min(max(value, self.forward_min), self.forward_max)
        elif value < self.neutral:
            return max(min(value, self.reverse_min), self.reverse_max)
        return self.neutral

    def __del__(self):
        """Cleanup resources"""
        self.left_motor.close()
        self.right_motor.close()