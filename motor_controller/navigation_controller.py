#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Create publisher for wheel speeds
        self.wheel_speeds_pub = self.create_publisher(
            Twist,
            'wheel_speeds',
            10
        )
        
        # Create subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Navigation controller initialized')

    def cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming velocity commands and pass through to wheel speeds.
        """
        # Just pass through the command directly
        self.wheel_speeds_pub.publish(msg)
        
        # Debug logging
        self.get_logger().debug(
            f'Passing through velocities:\n'
            f'  Linear: {msg.linear.x:.2f} m/s\n'
            f'  Angular: {msg.angular.z:.2f} rad/s'
        )

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
