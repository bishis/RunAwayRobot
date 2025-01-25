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
        """Convert cmd_vel to binary servo values (-1, 0, 1)"""
        # Create wheel speeds message
        wheel_speeds = Twist()
        
        # Default to neutral (0)
        wheel_speeds.linear.x = 0.0  # Speed channel
        wheel_speeds.angular.z = 0.0  # Turn channel
        
        # Handle linear motion first (forward/reverse)
        if abs(msg.linear.x) > self.linear_threshold:
            if msg.linear.x > 0:
                wheel_speeds.linear.x = 1.0  # Full forward
            else:
                wheel_speeds.linear.x = -1.0  # Full reverse
        
        # Handle angular motion (turning)
        if abs(msg.angular.z) > self.angular_threshold:
            if msg.angular.z > 0:
                wheel_speeds.angular.z = -1.0  # Full left turn
            else:
                wheel_speeds.angular.z = 1.0  # Full right turn
        
        # Debug logging
        self.get_logger().info(
            f'CMD_VEL Input:\n'
            f'  Linear: {msg.linear.x:6.3f} m/s, Angular: {msg.angular.z:6.3f} rad/s\n'
            f'Binary Servo Output:\n'
            f'  Speed Channel: {wheel_speeds.linear.x:4.1f} {"(FWD)" if wheel_speeds.linear.x > 0 else "(REV)" if wheel_speeds.linear.x < 0 else "(STOP)"}\n'
            f'  Turn Channel: {wheel_speeds.angular.z:4.1f} {"(RIGHT)" if wheel_speeds.angular.z > 0 else "(LEFT)" if wheel_speeds.angular.z < 0 else "(CENTER)"}'
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
