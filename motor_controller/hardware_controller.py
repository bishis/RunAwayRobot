#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from .controllers.motor_controller import MotorController

class HardwareController(Node):
    def __init__(self):
        super().__init__('hardware_controller')
        
        # Initialize motor controller
        self.motors = MotorController(
            left_pin=self.declare_parameter('left_pin', 18).value,
            right_pin=self.declare_parameter('right_pin', 12).value
        )
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        
        # Create a publisher to verify we're receiving commands
        self.status_pub = self.create_publisher(
            Twist,
            'motor_status',
            10
        )
        
        # Subscribe to LIDAR data to verify we're getting sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Create a timer for status updates
        self.create_timer(1.0, self.status_callback)
        
        self.get_logger().info('Hardware controller initialized')
        
    def velocity_callback(self, msg):
        """Convert velocity commands to motor speeds."""
        self.get_logger().info(f'Received cmd_vel - linear: {msg.linear.x}, angular: {msg.angular.z}')
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to motor speeds
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        self.get_logger().info(f'Setting motor speeds - left: {left_speed}, right: {right_speed}')
        
        # Apply to motors
        try:
            self.motors.set_speeds(left_speed, right_speed)
        except Exception as e:
            self.get_logger().error(f'Error setting motor speeds: {str(e)}')
    
    def scan_callback(self, msg):
        """Verify LIDAR data."""
        self.get_logger().debug(f'Received LIDAR scan with {len(msg.ranges)} points')
    
    def status_callback(self):
        """Publish status updates."""
        self.get_logger().info('Hardware controller is running')
        
    def __del__(self):
        """Cleanup on shutdown."""
        if hasattr(self, 'motors'):
            self.motors.stop()

def main(args=None):
    rclpy.init(args=args)
    controller = HardwareController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        controller.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        controller.motors.stop()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 