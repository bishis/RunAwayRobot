#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ..controllers.motor_controller import MotorController

class RobotHardwareNode(Node):
    def __init__(self):
        super().__init__('robot_hardware')
        
        # Initialize motor controller
        self.declare_parameter('left_motor_pin', 18)
        self.declare_parameter('right_motor_pin', 12)
        
        left_pin = self.get_parameter('left_motor_pin').value
        right_pin = self.get_parameter('right_motor_pin').value
        
        self.motors = MotorController(left_pin=left_pin, right_pin=right_pin)
        
        # Subscribe to movement commands
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Robot hardware node initialized')
        
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert velocity commands to motor actions
        if abs(angular_z) > 0.1:  # Turning
            if angular_z > 0:
                self.motors.turn_left()
            else:
                self.motors.turn_right()
        elif abs(linear_x) > 0.1:  # Moving forward/backward
            if linear_x > 0:
                self.motors.forward(speed=abs(linear_x))
            else:
                self.motors.backward()
        else:
            self.motors.stop()

def main(args=None):
    rclpy.init(args=args)
    node = RobotHardwareNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.motors.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 