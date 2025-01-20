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
        
        # Publishers for left and right track speeds (-1 to 1)
        self.left_track_pub = self.create_publisher(Float32, 'left_track_speed', 10)
        self.right_track_pub = self.create_publisher(Float32, 'right_track_speed', 10)
        
        # Robot parameters
        self.track_width = 0.3  # Distance between tracks in meters
        self.max_linear_speed = 0.1  # Max linear speed from nav2 params
        self.max_angular_speed = 0.5  # Max angular speed from nav2 params
        
    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to left and right track speeds
        # Using differential drive kinematics:
        # left = linear - angular * track_width/2
        # right = linear + angular * track_width/2
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Normalize to -1 to 1 range
        left_normalized = self.normalize_speed(left_speed)
        right_normalized = self.normalize_speed(right_speed)
        
        # Create and publish messages
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(left_normalized)
        right_msg.data = float(right_normalized)
        
        self.left_track_pub.publish(left_msg)
        self.right_track_pub.publish(right_msg)
        
        # Debug output
        self.get_logger().info(
            f'Converted cmd_vel({linear_x:.2f}, {angular_z:.2f}) to tracks L:{left_normalized:.2f} R:{right_normalized:.2f}'
        )
    
    def normalize_speed(self, speed: float) -> float:
        """Normalize speed to -1 to 1 range"""
        # Calculate the maximum possible speed (linear + angular contribution)
        max_possible_speed = self.max_linear_speed + (self.max_angular_speed * self.track_width / 2.0)
        
        # Normalize to -1 to 1
        normalized = speed / max_possible_speed
        
        # Clamp to -1 to 1
        return max(min(normalized, 1.0), -1.0)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
