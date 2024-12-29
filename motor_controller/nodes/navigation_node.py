#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ..processors.lidar_processor import LidarProcessor

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('safety_radius', 0.3),
                ('detection_distance', 0.5),
                ('leg_length', 2.0)
            ]
        )
        
        # Initialize processors
        self.lidar_processor = LidarProcessor(
            safety_radius=self.get_parameter('safety_radius').value,
            detection_distance=self.get_parameter('detection_distance').value
        )
        
        # Navigation state
        self.current_pose = None
        self.last_scan = None
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)
        
        # Control loop timer
        self.create_timer(0.05, self.control_loop)
        
    def control_loop(self):
        """Main control loop for navigation."""
        if self.current_pose is None or self.last_scan is None:
            return
            
        # Process LIDAR data
        sector_data = self.lidar_processor.process_scan(self.last_scan.ranges)
        if not sector_data:
            self.stop_robot()
            return
            
        # Get navigation command
        path_clear = self.lidar_processor.get_navigation_command(sector_data)
        
        # Create and publish movement command
        cmd = Twist()
        if path_clear:
            cmd.linear.x = 0.8  # Forward speed
        else:
            cmd.angular.z = 0.5  # Turn speed
            
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def lidar_callback(self, msg):
        """Process LIDAR data."""
        self.last_scan = msg
        
    def odom_callback(self, msg):
        """Process odometry data."""
        self.current_pose = msg.pose.pose

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 