#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

def test_motors():
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    
    # Motor pins
    LEFT_PIN = 18
    RIGHT_PIN = 12
    LEFT_DIR = 23
    RIGHT_DIR = 24
    
    # Setup pins
    GPIO.setup(LEFT_PIN, GPIO.OUT)
    GPIO.setup(RIGHT_PIN, GPIO.OUT)
    GPIO.setup(LEFT_DIR, GPIO.OUT)
    GPIO.setup(RIGHT_DIR, GPIO.OUT)
    
    # Create PWM objects
    left_pwm = GPIO.PWM(LEFT_PIN, 100)
    right_pwm = GPIO.PWM(RIGHT_PIN, 100)
    
    # Start PWM
    left_pwm.start(0)
    right_pwm.start(0)
    
    try:
        print("Testing forward motion...")
        GPIO.output(LEFT_DIR, GPIO.HIGH)
        GPIO.output(RIGHT_DIR, GPIO.HIGH)
        left_pwm.ChangeDutyCycle(50)
        right_pwm.ChangeDutyCycle(50)
        time.sleep(2)
        
        print("Stopping...")
        left_pwm.ChangeDutyCycle(0)
        right_pwm.ChangeDutyCycle(0)
        time.sleep(1)
        
        print("Testing turning left...")
        GPIO.output(LEFT_DIR, GPIO.LOW)
        GPIO.output(RIGHT_DIR, GPIO.HIGH)
        left_pwm.ChangeDutyCycle(50)
        right_pwm.ChangeDutyCycle(50)
        time.sleep(2)
        
    finally:
        left_pwm.stop()
        right_pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    test_motors() 