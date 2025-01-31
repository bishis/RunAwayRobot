#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)  # Half of 0.32m
        self.declare_parameter('safety_margin', 0.1)  # Additional safety distance
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Minimum safe distance (robot radius + safety margin)
        self.min_distance = self.robot_radius + self.safety_margin
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Store latest scan data
        self.latest_scan = None
        
        self.get_logger().info('Navigation controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        try:
            # Process velocities
            safe_linear = msg.linear.x
            safe_angular = msg.angular.z
                        
            # Create and publish wheel speeds message
            wheel_speeds = Twist()
            wheel_speeds.linear.x = safe_linear
            wheel_speeds.angular.z = safe_angular
            self.wheel_speeds_pub.publish(wheel_speeds)
            
            # Only log significant changes
            if abs(safe_linear - msg.linear.x) > 0.01 or abs(safe_angular - msg.angular.z) > 0.01:
                self.get_logger().info(
                    f'Modified speeds:\n'
                    f'  Linear: {msg.linear.x:.2f} -> {safe_linear:.2f}\n'
                    f'  Angular: {msg.angular.z:.2f} -> {safe_angular:.2f}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {str(e)}')
            # Stop robot on error
            stop_msg = Twist()
            self.wheel_speeds_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
