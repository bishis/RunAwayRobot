from gpiozero import Servo
import time
import RPi.GPIO as GPIO

class MotorController:
    """Handles low-level motor control operations using Servo controllers."""
    
    def __init__(self, left_pin, right_pin):
        """Initialize the motor controller."""
        self.left_pin = left_pin
        self.right_pin = right_pin
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_pin, GPIO.OUT)
        GPIO.setup(self.right_pin, GPIO.OUT)
        
        # Setup PWM
        self.left_pwm = GPIO.PWM(self.left_pin, 100)  # 100 Hz frequency
        self.right_pwm = GPIO.PWM(self.right_pin, 100)
        
        # Start PWM with 0% duty cycle
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
    def set_speeds(self, left_speed, right_speed):
        """Set motor speeds. Values should be between -1.0 and 1.0."""
        # Convert speeds to PWM duty cycle (0-100)
        left_duty = abs(left_speed) * 100
        right_duty = abs(right_speed) * 100
        
        # Ensure duty cycle is within bounds
        left_duty = min(100, max(0, left_duty))
        right_duty = min(100, max(0, right_duty))
        
        # Set PWM duty cycle
        self.left_pwm.ChangeDutyCycle(left_duty)
        self.right_pwm.ChangeDutyCycle(right_duty)
        
        print(f"Setting motor speeds - Left: {left_duty}%, Right: {right_duty}%")
        
    def stop(self):
        """Stop both motors."""
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 