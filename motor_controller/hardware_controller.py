#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Publishers for each wheel
        self.left_wheel_pub = self.create_publisher(Float32, 'left_wheel_speed', 10)
        self.right_wheel_pub = self.create_publisher(Float32, 'right_wheel_speed', 10)
        
        self.get_logger().info('Hardware Controller Node has been started')

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate wheel speeds (differential drive kinematics)
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        # Normalize to -1 or 1 based on direction
        left_speed = self.normalize_speed(left_speed)
        right_speed = self.normalize_speed(right_speed)
        
        # Create and publish messages
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(left_speed)
        right_msg.data = float(right_speed)
        
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)
        
        self.get_logger().debug(f'Left: {left_speed}, Right: {right_speed}')

    def normalize_speed(self, speed):
        """Normalize speed to -1 or 1 based on direction"""
        if speed > 0:
            return 1.0
        elif speed < 0:
            return -1.0
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
