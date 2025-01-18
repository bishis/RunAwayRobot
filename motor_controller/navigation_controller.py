#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot.radius', 0.27)
        self.declare_parameter('robot.safety_margin', 0.27)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot.radius').value
        self.safety_margin = self.get_parameter('robot.safety_margin').value
        
        # Publishers - publish to twist_mux topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        
        # Subscribers
        self.create_subscription(Twist, 'nav2/cmd_vel', self.nav2_cmd_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.get_logger().info('Navigation controller initialized')

    def nav2_cmd_callback(self, msg):
        """Convert Nav2 velocity commands to binary commands for the Pi."""
        try:
            # Create binary command
            cmd = Twist()
            
            # Convert linear velocity to binary (-1, 0, 1)
            if abs(msg.linear.x) < 0.1:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = 1.0 if msg.linear.x > 0 else -1.0
            
            # Convert angular velocity to binary (-1, 0, 1)
            if abs(msg.angular.z) < 0.1:
                cmd.angular.z = 0.0
            else:
                cmd.angular.z = 1.0 if msg.angular.z > 0 else -1.0
            
            # Publish binary command
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().debug(
                f'Converted velocity command - linear: {cmd.linear.x}, angular: {cmd.angular.z}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error converting velocity command: {str(e)}')

    def odom_callback(self, msg):
        """Handle odometry updates."""
        pass  # Nav2 will use this directly

    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        pass  # Nav2 will use this directly

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 