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
        
        # Add test movement timer
        self.create_timer(2.0, self.test_movement_callback)
        
        # Status variables
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        self.test_state = 0  # For cycling through test movements
        
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
            
            # For tank drive:
            # Pure rotation: One wheel forward, one backward
            # Pure translation: Both wheels same direction
            # Mixed: Combine both effects
            
            # Simpler tank drive calculation
            if abs(angular_z) < 0.001:
                # Pure straight motion
                left_speed = right_speed = linear_x
                self.get_logger().info('Pure straight motion')
            elif abs(linear_x) < 0.001:
                # Pure rotation
                left_speed = -angular_z
                right_speed = angular_z
                self.get_logger().info('Pure rotation')
            else:
                # Mixed motion
                left_speed = linear_x - angular_z
                right_speed = linear_x + angular_z
                self.get_logger().info('Mixed motion')
            
            # Normalize to [-1, 1] range
            max_speed = max(abs(left_speed), abs(right_speed))
            if max_speed > 1.0:
                left_speed = left_speed / max_speed
                right_speed = right_speed / max_speed
            
            # Log detailed movement info
            self.get_logger().info(
                f'Movement calculation:'
                f'\n  Movement type: {"rotation" if abs(angular_z) > abs(linear_x) else "translation"}'
                f'\n  Final wheel speeds [-1 to 1]:'
                f'\n    Left: {left_speed:.3f}'
                f'\n    Right: {right_speed:.3f}'
                f'\n  Expected behavior:'
                f'\n    Forward/Back: {abs(linear_x) > abs(angular_z)}'
                f'\n    Turning: {abs(angular_z) > abs(linear_x)}'
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
        
        # Log the basic debug info
        self.get_logger().info(
            f'\nDebug Info for {node_name}:'
            f'\n  Time since last cmd_vel: {time_since_cmd:.2f} seconds'
            f'\n  Number of publishers to cmd_vel: {n_publishers}'
            f'\n  Number of subscribers to wheel_cmd_vel: {n_subscribers}'
            f'\n  Subscribed to cmd_vel topic: {self.get_parameter("cmd_vel_topic").value}'
        )
        
        # Check if we have any publishers
        if n_publishers == 0:
            self.get_logger().warn('No publishers on cmd_vel topic - Nav2 may not be running properly')

    def test_movement_callback(self):
        """Send test movement commands to verify hardware chain."""
        # Create a test Twist message
        test_cmd = Twist()
        
        # Cycle through different movements
        if self.test_state == 0:
            # Forward
            test_cmd.linear.x = 0.2
            test_cmd.angular.z = 0.0
            self.get_logger().info('Test: Moving Forward')
        elif self.test_state == 1:
            # Turn left
            test_cmd.linear.x = 0.0
            test_cmd.angular.z = 0.5
            self.get_logger().info('Test: Turning Left')
        elif self.test_state == 2:
            # Turn right
            test_cmd.linear.x = 0.0
            test_cmd.angular.z = -0.5
            self.get_logger().info('Test: Turning Right')
        else:
            # Stop
            test_cmd.linear.x = 0.0
            test_cmd.angular.z = 0.0
            self.get_logger().info('Test: Stopping')
        
        # Process the test command as if it came from Nav2
        self.cmd_vel_callback(test_cmd)
        
        # Cycle to next test state
        self.test_state = (self.test_state + 1) % 4

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