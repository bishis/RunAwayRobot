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
        
        # Subscribe to velocity commands
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info('Hardware controller initialized')
        
    def velocity_callback(self, msg):
        """Convert velocity commands to motor speeds."""
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to motor speeds
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        # Apply to motors
        self.motors.set_speeds(left_speed, right_speed)
        
    def __del__(self):
        """Cleanup on shutdown."""
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
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 