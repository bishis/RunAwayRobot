#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import OutputDevice, PWMOutputDevice
import time
import signal
import sys

class MotorController:
    def __init__(self, 
                 left_dir_pin: int = 27,
                 left_pwm_pin: int = 18,
                 right_dir_pin: int = 17,
                 right_pwm_pin: int = 4,
                 pwm_frequency: int = 1000,
                 logger=None):
        """
        Initialize motor controller with configurable pins.
        
        Args:
            left_dir_pin: GPIO pin for left motors direction
            left_pwm_pin: GPIO pin for left motors PWM
            right_dir_pin: GPIO pin for right motors direction
            right_pwm_pin: GPIO pin for right motors PWM
            pwm_frequency: PWM frequency in Hz
            logger: ROS logger instance
        """
        # Store logger
        self.logger = logger
        
        # Left side motors
        self.left_dir = OutputDevice(left_dir_pin)
        self.left_pwm = PWMOutputDevice(left_pwm_pin, frequency=pwm_frequency)
        
        # Right side motors
        self.right_dir = OutputDevice(right_dir_pin)
        self.right_pwm = PWMOutputDevice(right_pwm_pin, frequency=pwm_frequency)
        
        # Initialize motors to stopped state
        self.stop_motors()
        
        # Register cleanup handler
        signal.signal(signal.SIGINT, self.cleanup)

    def set_speeds(self, linear: float, angular: float):
        """
        Update motor speeds based on linear and angular velocity commands.
        linear: Forward/backward speed (-1.0 to 1.0)
        angular: Left/right turning speed (-1.0 to 1.0)
        """
        # Clamp values to valid range
        linear = max(min(linear, 1.0), -1.0)
        angular = max(min(angular, 1.0), -1.0)
        
        # Increase turning sensitivity
        TURN_BOOST = 1.5  # Amplify turning effect
        angular *= TURN_BOOST
        
        # Calculate left and right motor speeds with enhanced turning
        if abs(angular) > 0.1:  # If turning
            # Prioritize turning by reducing forward speed
            linear *= 0.5  # Reduce forward speed during turns
            left_speed = linear - angular
            right_speed = linear + angular
        else:
            # Normal straight movement
            left_speed = linear - angular
            right_speed = linear + angular
        
        # Normalize speeds if they exceed [-1, 1]
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
        
        # Apply minimum threshold - if speed is non-zero but below threshold, set to threshold
        MIN_SPEED = 0.7  # 70% power minimum
        TURN_MIN_SPEED = 0.8  # Higher minimum speed for turning
        
        # Use higher minimum speed when turning
        current_min_speed = TURN_MIN_SPEED if abs(angular) > 0.1 else MIN_SPEED
        
        if abs(left_speed) > 0 and abs(left_speed) < current_min_speed:
            left_speed = current_min_speed if left_speed > 0 else -current_min_speed
            if self.logger:
                self.logger.info(f'Left speed boosted to minimum: {left_speed:.2f}')
        
        if abs(right_speed) > 0 and abs(right_speed) < current_min_speed:
            right_speed = current_min_speed if right_speed > 0 else -current_min_speed
            if self.logger:
                self.logger.info(f'Right speed boosted to minimum: {right_speed:.2f}')
        
        # Set left motors (reversed mounting)
        if left_speed >= 0:
            self.left_dir.off()  # Forward (reversed due to mounting)
            self.left_pwm.value = abs(left_speed)
        else:
            self.left_dir.on()   # Backward (reversed due to mounting)
            self.left_pwm.value = abs(left_speed)
            
        # Set right motors (reversed mounting)
        if right_speed >= 0:
            self.right_dir.on()  # Forward (reversed due to mounting)
            self.right_pwm.value = abs(right_speed)
        else:
            self.right_dir.off() # Backward (reversed due to mounting)
            self.right_pwm.value = abs(right_speed)
        
        # Log actual speeds being applied
        if self.logger:
            self.logger.info(
                f'Motor speeds - Left: {left_speed:6.3f}, Right: {right_speed:6.3f} (Turn boost: {abs(angular) > 0.1})'
            )

    def stop_motors(self):
        """Stop all motors"""
        self.left_pwm.value = 0
        self.right_pwm.value = 0

    def cleanup(self, signal, frame):
        """Cleanup function to stop motors on shutdown"""
        self.stop_motors()
        # Allow time for motors to stop
        time.sleep(0.1)
        sys.exit(0)

    def __del__(self):
        """Ensure motors are stopped when object is destroyed"""
        self.stop_motors()


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # Declare parameters
        self.declare_parameter('left_dir_pin', 27)
        self.declare_parameter('left_pwm_pin', 18)
        self.declare_parameter('right_dir_pin', 17)
        self.declare_parameter('right_pwm_pin', 4)
        self.declare_parameter('pwm_frequency', 1000)
        
        # Get parameters
        left_dir_pin = self.get_parameter('left_dir_pin').value
        left_pwm_pin = self.get_parameter('left_pwm_pin').value
        right_dir_pin = self.get_parameter('right_dir_pin').value
        right_pwm_pin = self.get_parameter('right_pwm_pin').value
        pwm_frequency = self.get_parameter('pwm_frequency').value
        
        # Create motor controller with parameters
        self.motor_controller = MotorController(
            left_dir_pin=left_dir_pin,
            left_pwm_pin=left_pwm_pin,
            right_dir_pin=right_dir_pin,
            right_pwm_pin=right_pwm_pin,
            pwm_frequency=pwm_frequency,
            logger=self.get_logger()
        )
        
        # Create subscriber for wheel speeds
        self.subscription = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        self.get_logger().info(
            f'Motor controller initialized with pins:\n'
            f'  Left:  DIR={left_dir_pin}, PWM={left_pwm_pin}\n'
            f'  Right: DIR={right_dir_pin}, PWM={right_pwm_pin}\n'
            f'  PWM Frequency: {pwm_frequency}Hz'
        )

    def wheel_speeds_callback(self, msg: Twist):
        """Handle incoming wheel speed commands"""
        try:
            # Convert Twist message to motor commands
            linear = msg.linear.x   # Forward/backward
            angular = msg.angular.z  # Left/right turning
            
            # Update motor speeds
            self.motor_controller.set_speeds(linear, angular)
            
            # Debug logging
            self.get_logger().debug(
                f'Motors: linear={linear:.2f}, angular={angular:.2f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error controlling motors: {str(e)}')
            # Try to stop motors on error
            self.motor_controller.stop_motors()

    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'motor_controller'):
            self.motor_controller.stop_motors()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()