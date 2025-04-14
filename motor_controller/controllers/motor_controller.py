#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import OutputDevice, PWMOutputDevice
import time
import signal
import sys
import math

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
        
        # Robot physical parameters - using actual measurements
        self.wheel_width = 0.3429  # Overall width including wheels (13.50 inches) in meters
        self.wheel_diameter = 0.12065  # Wheel diameter (4.75 inches) in meters
        self.wheel_radius = self.wheel_diameter / 2.0
        
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

    def scale_motor_speeds(self, left_speed: float, right_speed: float, angular: float) -> tuple[float, float]:
        MIN_SPEED = 0.84  # A lower base value for turning; tune as needed

        # Find the absolute speeds.
        abs_left, abs_right = abs(left_speed), abs(right_speed)
        
        # Determine if both speeds are nonzero and below the minimum
        if left_speed != 0 and abs_left < MIN_SPEED:
            left_speed = (left_speed/abs_left) * MIN_SPEED
        if right_speed != 0 and abs_right < MIN_SPEED:
            right_speed = (right_speed/abs_right) * MIN_SPEED
        
        # Optionally, apply normalization if any speed exceeds 1.0.
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
            
        return left_speed, right_speed
                

    def set_speeds(self, linear: float, angular: float) -> tuple[float, float, float, float]:
        """
        Update motor speeds based on linear and angular velocity commands.
        """
        # Convert to differential drive using the correct formula
        left_speed = linear + (angular * self.wheel_width / 2.0)
        right_speed = linear - (angular * self.wheel_width / 2.0)
        
        # Apply scaling and minimum speeds
        left_speed, right_speed = self.scale_motor_speeds(left_speed, right_speed, angular)
        
        # Store PWM values
        left_pwm = abs(left_speed)
        right_pwm = abs(right_speed)
        
        # Set motor directions and speeds
        if left_speed >= 0:
            self.left_dir.off()  # Forward for motor = Backward for robot
            self.left_pwm.value = left_pwm
        else:
            self.left_dir.on()   # Backward for motor = Forward for robot
            self.left_pwm.value = left_pwm
            
        if right_speed >= 0:
            self.right_dir.on()  # Forward for motor = Backward for robot
            self.right_pwm.value = right_pwm
        else:
            self.right_dir.off() # Backward for motor = Forward for robot
            self.right_pwm.value = right_pwm
        
        return left_speed, right_speed, left_pwm, right_pwm

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
            left_speed, right_speed, left_pwm, right_pwm = self.motor_controller.set_speeds(linear, angular)
            
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