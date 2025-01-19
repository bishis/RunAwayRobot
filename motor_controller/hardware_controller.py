#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.declare_parameter('left_pin', 18).value,
            right_pin=self.declare_parameter('right_pin', 12).value
        )
        
        # Subscribe to Nav2's velocity commands
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
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
        
    def velocity_callback(self, msg):
        """Convert Nav2 velocity commands to motor speeds."""
        try:
            # Extract linear and angular velocities
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Convert to wheel speeds
            left_speed = linear_x - angular_z
            right_speed = linear_x + angular_z
            
            # Print received command details
            self.get_logger().info(
                f"\nReceived cmd_vel:"
                f"\n  Linear X: {linear_x:.2f}"
                f"\n  Angular Z: {angular_z:.2f}"
                f"\nCalculated wheel speeds:"
                f"\n  Left: {left_speed:.2f}"
                f"\n  Right: {right_speed:.2f}"
            )
            
            # Apply to motors
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