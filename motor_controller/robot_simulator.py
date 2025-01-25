#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update = time.time()
        
        # Robot parameters
        self.declare_parameter('wheel_separation', 0.24)  # meters
        self.declare_parameter('max_linear_speed', 0.1)   # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # Subscribe to wheel speeds
        self.subscription = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Update timer (50Hz)
        self.create_timer(0.02, self.update_pose)
        
        self.get_logger().info('Robot simulator initialized')

    def wheel_speeds_callback(self, msg: Twist):
        """Store the current wheel speeds"""
        self.current_linear = msg.linear.x * self.get_parameter('max_linear_speed').value
        self.current_angular = -msg.angular.z * self.get_parameter('max_angular_speed').value  # Negative for correct rotation direction

    def update_pose(self):
        """Update robot pose based on current speeds"""
        current_time = time.time()
        dt = current_time - self.last_update
        
        if hasattr(self, 'current_linear') and hasattr(self, 'current_angular'):
            # Update pose
            if abs(self.current_angular) < 0.0001:  # Straight line motion
                self.x += self.current_linear * math.cos(self.theta) * dt
                self.y += self.current_linear * math.sin(self.theta) * dt
            else:  # Arc motion
                self.theta += self.current_angular * dt
                self.x += self.current_linear * math.cos(self.theta) * dt
                self.y += self.current_linear * math.sin(self.theta) * dt
            
            # Normalize theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Publish odometry
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Set position
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = math.sin(self.theta/2.0)
            odom.pose.pose.orientation.w = math.cos(self.theta/2.0)
            
            # Set velocity
            odom.twist.twist.linear.x = self.current_linear
            odom.twist.twist.angular.z = self.current_angular
            
            # Publish odometry
            self.odom_pub.publish(odom)
            
            # Broadcast transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(self.theta/2.0)
            t.transform.rotation.w = math.cos(self.theta/2.0)
            
            self.tf_broadcaster.sendTransform(t)
        
        self.last_update = current_time

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 