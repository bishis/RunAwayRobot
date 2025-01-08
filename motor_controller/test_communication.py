#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Test publisher started')
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward at half speed
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Published test message')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 