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

    import math
    from typing import Tuple

    def scale_motor_speeds(
        self,
        left_speed: float,          # raw wheel speeds from the planner, **metres s‑1**
        right_speed: float,
        angular: float,             # the planner’s angular command, **rad s‑1**
        *,
        min_vel_x: float   = -0.05, # planner limits (m s‑1)
        max_vel_x: float   =  0.10,
        min_vel_theta: float = 0.20,# planner limits (rad s‑1)
        max_vel_theta: float = 1.30,
        MIN_PWM: float = 0.82       # 0.82 means “82 % throttle is the lowest that
    ) -> Tuple[float, float]:       #   will actually turn the motor”
        """
        Convert *physical* wheel‑speed requests (m s‑1) into the normalised
        [-1 … 1] range accepted by your motor driver.

        The procedure is:

        1.  Express each wheel speed as a fraction of the planner’s
            **maximum possible** wheel speed.
        2.  If both fractions are ≈ 0, return 0, 0 (don’t force a minimum).
        3.  Bring the *slower* wheel up to at least MIN_PWM of full power,
            then scale the *faster* wheel by the **same factor** so the
            left / right ratio is preserved.
        4.  If that pushed anything over |1|, renormalise the pair.
        """

        # --- 1. Normalise the physical speeds to the planner’s extremes -----------
        # In the worst case (spin‑in‑place) one wheel will see +max_vel_theta
        # and the other −max_vel_theta, so that sets our per‑wheel ceiling.
        # For straight motion it is ±max_vel_x.
        # Take whichever is larger in magnitude for each wheel.
        max_wheel_mag = max(abs(max_vel_x), abs(min_vel_x),
                            abs(max_vel_theta))          # → m s‑1

        nl = left_speed  / max_wheel_mag   # now in [‑1 … 1] (but maybe smaller)
        nr = right_speed / max_wheel_mag

        # --- 2. If both wheels are (almost) stopped, get out early ---------------
        if abs(nl) < 1e-6 and abs(nr) < 1e-6:
            return 0.0, 0.0

        # --- 3. Raise the slower wheel to MIN_PWM while preserving the ratio -----
        slow_mag = min(abs(nl), abs(nr))
        # Handle the “one wheel exactly zero” edge‑case:
        #   • If one wheel is already zero we *still* want the other wheel
        #     to be ≥ MIN_PWM, otherwise the robot sits there buzzing.
        if slow_mag == 0.0:
            factor = MIN_PWM / max(abs(nl), abs(nr))
        else:
            factor = max(1.0, MIN_PWM / slow_mag)        # never scale < 1 ×

        new_l = math.copysign(abs(nl) * factor, nl)
        new_r = math.copysign(abs(nr) * factor, nr)

        # --- 4. Final clip in case we crossed the ±1 boundary ---------------------
        peak = max(abs(new_l), abs(new_r))
        if peak > 1.0:
            new_l /= peak
            new_r /= peak

        return new_l, new_r

                

    def set_speeds(self, linear: float, angular: float) -> tuple[float, float, float, float]:
        """
        Update motor speeds based on linear and angular velocity commands.
        """
        # Convert to differential drive using the correct formula
        left_speed = linear + (angular * self.wheel_width / 2.0)
        right_speed = linear - (angular * self.wheel_width / 2.0)
        
        # Apply scaling and minimum speeds
        left_speed, right_speed = self.scale_motor_speeds(left_speed, right_speed, angular=angular)
        
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