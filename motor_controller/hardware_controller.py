#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_controller.controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.declare_parameter('left_pin', 18).value,
            right_pin=self.declare_parameter('right_pin', 12).value
        )
        
        # Subscribe to wheel speed commands with explicit QoS
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # Make sure to use absolute topic name
            self.wheel_velocity_callback,
            10
        )
        
        # Add counter for received commands
        self.command_count = 0
        self.last_cmd_time = self.get_clock().now()
        
        self.get_logger().info('Hardware controller initialized and ready for commands')
        
        # Create a timer to print status more frequently
        self.create_timer(0.5, self.status_callback)
        
    def convert_to_binary_speed(self, speed):
        """Convert decimal speed to binary (0 or 1)."""
        threshold = 0.1  # Threshold for movement
        if abs(speed) < threshold:
            return 0
        return 1 if speed > 0 else -1
        
    def wheel_velocity_callback(self, msg):
        """Handle incoming wheel velocity commands."""
        try:
            # Convert Twist to differential drive commands
            linear_x = msg.linear.x    # Forward/backward motion
            angular_z = msg.angular.z   # Rotational motion
            
            # Calculate wheel speeds for differential drive
            # Track width is the distance between wheels (adjust as needed)
            track_width = 0.3  # meters
            
            # Convert twist to differential drive
            left_speed = linear_x - (angular_z * track_width / 2.0)
            right_speed = linear_x + (angular_z * track_width / 2.0)
            
            # Print received command details
            self.get_logger().info(
                f"\nReceived cmd_vel:"
                f"\n  Linear X: {linear_x:.3f} m/s"
                f"\n  Angular Z: {angular_z:.3f} rad/s"
                f"\nCalculated wheel speeds:"
                f"\n  Left: {left_speed:.3f}"
                f"\n  Right: {right_speed:.3f}"
            )
            
            # Convert to binary speeds (-1, 0, 1)
            left_binary = self.convert_to_binary_speed(left_speed)
            right_binary = self.convert_to_binary_speed(right_speed)
            
            # Apply speeds to motors
            self.motors.set_speeds(left_binary, right_binary)
            
            # Increment command counter
            self.command_count += 1
            self.last_cmd_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f'Error setting motor speeds: {str(e)}')
    
    def status_callback(self):
        """Print periodic status updates."""
        now = self.get_clock().now()
        time_since_last = (now - self.last_cmd_time).nanoseconds / 1e9
        
        self.get_logger().info(
            f'Status:'
            f'\n  Total commands received: {self.command_count}'
            f'\n  Time since last command: {time_since_last:.1f}s'
            f'\n  Subscribers to cmd_vel: {self.cmd_vel_sub.get_publisher_count()}'
        )
    
    def __del__(self):
        if hasattr(self, 'motors'):
            self.motors.stop()

def main(args=None):
    rclpy.init(args=args)
    controller = HardwareController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.motors.stop()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
