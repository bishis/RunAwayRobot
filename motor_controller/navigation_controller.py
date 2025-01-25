#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters for speed conversion
        self.declare_parameter('max_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('linear_threshold', 0.01)
        self.declare_parameter('angular_threshold', 0.02)
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        
        # Create publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Navigation controller initialized')

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to speed and turn commands"""
        # Clamp velocities to max values
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        # Convert to normalized speed and turn commands (-1 to 1)
        speed = 0.0
        turn = 0.0
        
        # Handle linear motion
        if abs(linear_x) > self.linear_threshold:
            speed = linear_x / self.max_linear_speed
            
        # Handle angular motion
        if abs(angular_z) > self.angular_threshold:
            turn = angular_z / self.max_angular_speed
            
        # Create wheel speeds message
        wheel_speeds = Twist()
        
        # Convert to PWM-like values (0.05-0.10 range)
        # For speed channel: 0.075 is neutral, 0.09 is full forward, 0.045 is full reverse
        # For turn channel: 0.075 is neutral, 0.09 is full right, 0.045 is full left
        wheel_speeds.linear.x = 0.075 + (speed * 0.015)  # Speed channel
        wheel_speeds.angular.z = 0.075 + (turn * 0.015)  # Turn channel
        
        # Debug logging
        self.get_logger().info(
            f'CMD_VEL Input:\n'
            f'  Linear: {linear_x:6.3f} m/s, Angular: {angular_z:6.3f} rad/s\n'
            f'Normalized Commands:\n'
            f'  Speed: {speed:6.3f}, Turn: {turn:6.3f}\n'
            f'PWM Output:\n'
            f'  Speed Channel: {wheel_speeds.linear.x:.4f}\n'
            f'  Turn Channel: {wheel_speeds.angular.z:.4f}'
        )
        
        # Publish wheel speeds
        self.wheel_speeds_pub.publish(wheel_speeds)

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
