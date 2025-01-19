#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Nav2HardwareBridge(Node):
    """Bridge between Nav2 commands and hardware controller."""
    
    def __init__(self):
        super().__init__('nav2_hardware_bridge')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel_smoothed',
            self.cmd_vel_callback,
            10
        )
        
        self.wheel_cmd_pub = self.create_publisher(
            Twist,
            'wheel_cmd_vel',
            10
        )
        
        # Status variables
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        
        # Create watchdog timer
        self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info('Nav2 Hardware Bridge initialized')
        
    def cmd_vel_callback(self, msg: Twist):
        """Convert Nav2 velocity commands to wheel velocities."""
        try:
            # Get linear and angular velocities
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Clamp velocities to max values
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            
            # Convert to differential drive commands
            # Similar to your navigation_controller implementation
            left_speed = linear_x - (angular_z * 0.1)  # 0.1 is wheel separation/2
            right_speed = linear_x + (angular_z * 0.1)
            
            # Create wheel command message
            wheel_cmd = Twist()
            wheel_cmd.linear.x = left_speed   # Using linear.x for left wheel
            wheel_cmd.linear.y = right_speed  # Using linear.y for right wheel
            
            # Publish wheel commands
            self.wheel_cmd_pub.publish(wheel_cmd)
            
            # Update command time
            self.last_cmd_time = self.get_clock().now()
            
            # Log command details
            self.get_logger().debug(
                f'Nav2 cmd: linear={linear_x:.2f}, angular={angular_z:.2f} -> '
                f'wheels: left={left_speed:.2f}, right={right_speed:.2f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing velocity command: {str(e)}')
            
    def watchdog_callback(self):
        """Stop robot if no commands received recently."""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > self.cmd_timeout:
            # Send stop command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0  # Left wheel
            stop_cmd.linear.y = 0.0  # Right wheel
            self.wheel_cmd_pub.publish(stop_cmd)

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