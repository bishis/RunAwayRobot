import RPi.GPIO as GPIO

class MotorController:
    """Handles low-level motor control operations for L298N or similar motor drivers."""
    
    def __init__(self, left_pin=18, right_pin=12):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Store pin numbers
        self.LEFT_PIN = left_pin
        self.RIGHT_PIN = right_pin
        
        # Setup pins
        GPIO.setup(self.LEFT_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_PIN, GPIO.OUT)
        
        # Setup PWM
        self.left_pwm = GPIO.PWM(self.LEFT_PIN, 1000)  # 1000 Hz frequency
        self.right_pwm = GPIO.PWM(self.RIGHT_PIN, 1000)
        
        # Start PWM with 0% duty cycle
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        # Initialize velocities
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
    def set_speeds(self, left_speed, right_speed):
        """
        Set motor speeds between -1 and 1.
        Positive values for forward, negative for reverse.
        """
        # Convert speeds to PWM duty cycle (0-100)
        left_pwm = abs(left_speed) * 100
        right_pwm = abs(right_speed) * 100
        
        # Apply PWM values
        self.left_pwm.ChangeDutyCycle(left_pwm)
        self.right_pwm.ChangeDutyCycle(right_pwm)
        
        # Store velocities
        self.left_wheel_vel = left_speed
        self.right_wheel_vel = right_speed
        
    def stop(self):
        """Stop both motors."""
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
    def __del__(self):
        """Cleanup GPIO on object destruction."""
        try:
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup([self.LEFT_PIN, self.RIGHT_PIN])
        except:
            pass  # Ignore cleanup errors 