#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Nav2HardwareBridge(Node):
    """Simple bridge between Nav2 commands and hardware controller."""
    
    def __init__(self):
        super().__init__('nav2_hardware_bridge')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',  # Subscribe to Nav2's output
            self.cmd_vel_callback,
            10
        )
        
        self.wheel_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # Publish to hardware controller
            10
        )
        
        # Debug timer
        self.create_timer(1.0, self.debug_callback)
        self.command_count = 0
        
        self.get_logger().info('Nav2 Hardware Bridge initialized')
        
    def cmd_vel_callback(self, msg: Twist):
        """Convert Nav2 velocity commands to wheel velocities."""
        try:
            # Get linear and angular velocities
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            self.get_logger().info(
                f"\nReceived Nav2 command:"
                f"\n  Linear X: {linear_x}"
                f"\n  Angular Z: {angular_z}"
            )
            
            # Convert to differential drive commands
            # When turning, one wheel forward, one backward
            if abs(angular_z) > 0.1:
                left_speed = -angular_z
                right_speed = angular_z
            # When moving straight, both wheels same direction
            else:
                left_speed = linear_x
                right_speed = linear_x
            
            # Create wheel command message
            wheel_cmd = Twist()
            wheel_cmd.linear.x = float(left_speed)   # Left wheel
            wheel_cmd.linear.y = float(right_speed)  # Right wheel
            
            # Log the command being sent
            self.get_logger().info(
                f"Sending wheel commands:"
                f"\n  Left: {left_speed}"
                f"\n  Right: {right_speed}"
            )
            
            # Publish wheel commands
            self.wheel_cmd_pub.publish(wheel_cmd)
            self.command_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing velocity command: {str(e)}')
            
    def debug_callback(self):
        """Print debug info."""
        self.get_logger().info(
            f'Status:'
            f'\n  Commands processed: {self.command_count}'
            f'\n  Publishers to cmd_vel: {self.cmd_vel_sub.get_publisher_count()}'
            f'\n  Subscribers to wheel cmd: {self.wheel_cmd_pub.get_subscription_count()}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Nav2HardwareBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()