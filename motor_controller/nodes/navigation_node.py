#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ..processors.lidar_processor import LidarProcessor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Define QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Add heartbeat subscription with reliable QoS
        self.robot_online = False
        self.last_heartbeat = None
        self.create_subscription(
            String,
            'robot_heartbeat',
            self.heartbeat_callback,
            qos_profile
        )
        
        # Add command echo subscription with reliable QoS
        self.create_subscription(
            String,
            'cmd_echo',
            self.cmd_echo_callback,
            qos_profile
        )
        
        # Add status check timer
        self.create_timer(2.0, self.check_robot_status)
        
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
        
        # Publishers with reliable QoS
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile
        )
        
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
        
    def heartbeat_callback(self, msg):
        self.robot_online = True
        self.last_heartbeat = self.get_clock().now()
        self.get_logger().info(f"Received robot heartbeat: {msg.data}")
        
    def cmd_echo_callback(self, msg):
        self.get_logger().info(f"Command confirmation: {msg.data}")
        
    def check_robot_status(self):
        if not self.robot_online:
            self.get_logger().warn("No heartbeat received from robot yet!")
            return
            
        if self.last_heartbeat:
            time_since_heartbeat = (self.get_clock().now() - self.last_heartbeat).nanoseconds / 1e9
            if time_since_heartbeat > 5.0:
                self.get_logger().warn(f"No heartbeat for {time_since_heartbeat:.1f} seconds!")
                self.robot_online = False

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 