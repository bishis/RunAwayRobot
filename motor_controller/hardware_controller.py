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
        
        # Subscribe to wheel speed commands
        self.create_subscription(
            Twist,
            '/wheel_cmd_vel',
            self.wheel_velocity_callback,
            10
        )
        
        # Add counter for received commands
        self.command_count = 0
        
        self.get_logger().info('Hardware controller initialized and ready for commands')
        
        # Create a timer to print status
        self.create_timer(1.0, self.status_callback)
        
    def convert_to_binary_speed(self, speed):
        """Convert decimal speed to binary (0 or 1)."""
        threshold = 0.1  # Threshold for movement
        if abs(speed) < threshold:
            return 0
        return 1 if speed > 0 else -1
        
    def wheel_velocity_callback(self, msg):
        """Handle incoming wheel velocity commands."""
        try:
            # Extract wheel speeds from Twist message
            left_speed = msg.linear.x   # Left wheel speed
            right_speed = msg.linear.y  # Right wheel speed
            
            # Print received command details
            self.get_logger().info(
                f"\nReceived wheel speeds:"
                f"\n  Left: {left_speed:.3f}"
                f"\n  Right: {right_speed:.3f}"
            )
            
            # Apply speeds to motors
            self.motors.set_speeds(left_speed, right_speed)
            
            # Increment command counter
            self.command_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error setting motor speeds: {str(e)}')
    
    def status_callback(self):
        """Print periodic status updates."""
        self.get_logger().info(f'Total commands received: {self.command_count}')
    
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
