#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from .controllers.motor_controller import MotorController


class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        self.get_logger().info('Hardware Controller Starting...')
        self.motors = MotorController(
            left_pin=self.declare_parameter('left_pin', 18).value,
            right_pin=self.declare_parameter('right_pin', 12).value
        )        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Subscribed to cmd_vel')
        
        # Publishers for left and right track speeds (-1 to 1)
        self.left_track_pub = self.create_publisher(Float32, 'left_track_speed', 10)
        self.right_track_pub = self.create_publisher(Float32, 'right_track_speed', 10)
        self.get_logger().info('Publishers created for track speeds')
        
        # Robot parameters
        self.track_width = 0.3  # Distance between tracks in meters
        self.max_linear_speed = 0.1  # Max linear speed from nav2 params
        self.max_angular_speed = 0.5  # Max angular speed from nav2 params
        
        # Add a timer to report if we're receiving commands
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(1.0, self.check_cmd_vel)
    
    def check_cmd_vel(self):
        """Report if we haven't received a command in a while"""
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 1.0:  # More than 1 second
            self.get_logger().warn('No cmd_vel received in last second')
    
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Log raw command
        self.get_logger().info(f'Received cmd_vel - linear: {linear_x:.3f}, angular: {angular_z:.3f}')
        
        # Convert to left and right track speeds
        left_speed = linear_x - (angular_z * self.track_width / 2.0)
        right_speed = linear_x + (angular_z * self.track_width / 2.0)
        
        # Log pre-normalized speeds
        self.get_logger().info(f'Pre-normalized speeds - L: {left_speed:.3f}, R: {right_speed:.3f}')
        
        # Normalize to -1 to 1 range
        left_normalized = self.normalize_speed(left_speed)
        right_normalized = self.normalize_speed(right_speed)
        
        if left_normalized > 0:
            left_normalized = 1
        if right_normalized > 0:
            right_normalized = 1
        if left_normalized < 0:
            left_normalized = -1
        if right_normalized < 0:
            right_normalized = -1
        
        # Create and publish messages
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(left_normalized)
        right_msg.data = float(right_normalized)
        
        self.left_track_pub.publish(left_msg)
        self.right_track_pub.publish(right_msg)
        self.motors.set_speeds(left_normalized, right_normalized)

        # Final output
        self.get_logger().info(
            f'Published track speeds - L:{left_normalized:.2f} R:{right_normalized:.2f}'
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
