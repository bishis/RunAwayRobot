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
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')  # Make topic configurable
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Log subscribed topics
        self.get_logger().info(
            f'\nTopic Configuration:'
            f'\n  Subscribing to cmd_vel topic: {cmd_vel_topic}'
            f'\n  Publishing to wheel_cmd_vel topic: wheel_cmd_vel'
        )
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        self.wheel_cmd_pub = self.create_publisher(
            Twist,
            'wheel_cmd_vel',
            10
        )
        
        # Add debug timer to check cmd_vel and print topic info
        self.create_timer(1.0, self.debug_callback)
        
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
            
            # Log the received command
            self.get_logger().info(
                f'\nReceived cmd_vel:'
                f'\n  Linear X: {linear_x:.3f} m/s'
                f'\n  Angular Z: {angular_z:.3f} rad/s'
            )
            
            # Clamp velocities to max values
            original_linear = linear_x
            original_angular = angular_z
            linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            
            if original_linear != linear_x or original_angular != angular_z:
                self.get_logger().info(
                    f'Clamped velocities:'
                    f'\n  Linear X: {linear_x:.3f} m/s (from {original_linear:.3f})'
                    f'\n  Angular Z: {angular_z:.3f} rad/s (from {original_angular:.3f})'
                )
            
            # Tank drive conversion
            wheel_separation = 0.2  # Distance between wheels in meters
            
            # Calculate wheel speeds for tank drive
            if abs(angular_z) > 0.001:  # If we're turning
                # Tank drive turning: opposite wheel directions
                turn_speed = (angular_z * wheel_separation) / 2.0
                left_speed = linear_x - turn_speed
                right_speed = linear_x + turn_speed
                
                self.get_logger().info(
                    f'Turn calculation:'
                    f'\n  Turn speed component: {turn_speed:.3f}'
                    f'\n  Pre-normalized speeds:'
                    f'\n    Left: {left_speed:.3f}'
                    f'\n    Right: {right_speed:.3f}'
                )
            else:
                # Straight motion
                left_speed = right_speed = linear_x
                self.get_logger().info(
                    f'Straight motion - pre-normalized speed: {linear_x:.3f}'
                )
            
            # Normalize wheel speeds to [-1, 1]
            max_speed = max(abs(left_speed), abs(right_speed), 1.0)
            original_left = left_speed
            original_right = right_speed
            left_speed = left_speed / max_speed
            right_speed = right_speed / max_speed
            
            # Log the calculated wheel speeds
            self.get_logger().info(
                f'Wheel speeds:'
                f'\n  Pre-normalization:'
                f'\n    Left: {original_left:.3f}'
                f'\n    Right: {original_right:.3f}'
                f'\n  Normalization factor: {max_speed:.3f}'
                f'\n  Final speeds [-1 to 1]:'
                f'\n    Left: {left_speed:.3f}'
                f'\n    Right: {right_speed:.3f}'
            )
            
            # Create wheel command message
            wheel_cmd = Twist()
            wheel_cmd.linear.x = left_speed   # Left wheel (-1 to 1)
            wheel_cmd.linear.y = right_speed  # Right wheel (-1 to 1)
            
            # Publish wheel commands
            self.wheel_cmd_pub.publish(wheel_cmd)
            
            # Update command time
            self.last_cmd_time = self.get_clock().now()
            
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

    def debug_callback(self):
        """Debug timer callback to check if we're receiving cmd_vel."""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        # Get publisher and subscriber counts
        n_publishers = self.cmd_vel_sub.get_publisher_count()
        n_subscribers = self.wheel_cmd_pub.get_subscription_count()
        
        # Get node name info
        node_name = self.get_name()
        
        self.get_logger().info(
            f'\nDebug Info for {node_name}:'
            f'\n  Time since last cmd_vel: {time_since_cmd:.2f} seconds'
            f'\n  Number of publishers to cmd_vel: {n_publishers}'
            f'\n  Number of subscribers to wheel_cmd_vel: {n_subscribers}'
            f'\n  Subscribed to cmd_vel topic: {self.get_parameter("cmd_vel_topic").value}'
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