from gpiozero import Servo
import time
import RPi.GPIO as GPIO

class MotorController:
    """Handles low-level motor control operations using Servo controllers."""
    
    def __init__(self, left_pin, right_pin):
        """Initialize the motor controller."""
        self.left_pin = left_pin
        self.right_pin = right_pin
        
        # Add direction pins
        self.left_dir_pin = 23  # GPIO23
        self.right_dir_pin = 24  # GPIO24
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_pin, GPIO.OUT)
        GPIO.setup(self.right_pin, GPIO.OUT)
        GPIO.setup(self.left_dir_pin, GPIO.OUT)
        GPIO.setup(self.right_dir_pin, GPIO.OUT)
        
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
        
        # Set GPIO direction pins based on speed sign
        if left_speed >= 0:
            GPIO.output(self.left_dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.left_dir_pin, GPIO.LOW)
            
        if right_speed >= 0:
            GPIO.output(self.right_dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.right_dir_pin, GPIO.LOW)
        
        # Set PWM duty cycle
        self.left_pwm.ChangeDutyCycle(left_duty)
        self.right_pwm.ChangeDutyCycle(right_duty)
        
        print(f"Setting motor speeds - Left: {left_duty}% {'FWD' if left_speed >= 0 else 'REV'}, "
              f"Right: {right_duty}% {'FWD' if right_speed >= 0 else 'REV'}")
        
    def stop(self):
        """Stop both motors."""
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        
    def __del__(self):
        """Cleanup on object destruction."""
        self.stop() 