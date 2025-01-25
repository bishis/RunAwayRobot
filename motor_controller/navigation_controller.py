#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np

class SimpleNavigationController(Node):
    def __init__(self):
        super().__init__('simple_navigation_controller')
        
        # Motion control parameters
        self.declare_parameter('wheel_separation', 0.24)
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('linear_threshold', 0.05)
        self.declare_parameter('angular_threshold', 0.1)
        self.declare_parameter('speed_exponent', 2.0)
        
        # PWM parameters needed for mapping speeds
        self.declare_parameter('forward_max_duty', 0.09)
        self.declare_parameter('forward_min_duty', 0.09)
        self.declare_parameter('reverse_max_duty', 0.045)
        self.declare_parameter('reverse_min_duty', 0.045)
        self.declare_parameter('neutral_duty', 0.075)
        
        # Get parameters
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.speed_exponent = self.get_parameter('speed_exponent').value
        
        # Publishers and Subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(
            'Navigation controller initialized\n'
            f'Max speeds - Linear: {self.max_linear_speed} m/s, Angular: {self.max_angular_speed} rad/s\n'
            f'Wheel separation: {self.wheel_separation} m'
        )

    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose: {ex}')
            return None

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist commands to wheel speeds using binary control"""
        # Clamp input values
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # Get PWM values
        neutral = self.get_parameter('neutral_duty').value
        forward = self.get_parameter('forward_min_duty').value  # Use min values for consistent speed
        reverse = self.get_parameter('reverse_min_duty').value

        # Create wheel speeds message
        wheel_speeds = Twist()

        # Binary control strategy:
        # For turning, one wheel moves while the other stops
        # For straight motion, both wheels move at the same speed
        # For spot turns, wheels move in opposite directions

        if abs(angular_z) > self.angular_threshold:  # Turning is priority
            if angular_z > 0:  # CCW turn
                if linear_x > self.linear_threshold:  # Forward + left turn
                    wheel_speeds.linear.x = neutral    # Left wheel stop
                    wheel_speeds.angular.z = forward   # Right wheel forward
                elif linear_x < -self.linear_threshold:  # Reverse + left turn
                    wheel_speeds.linear.x = reverse    # Left wheel reverse
                    wheel_speeds.angular.z = neutral   # Right wheel stop
                else:  # Spot turn CCW
                    wheel_speeds.linear.x = reverse    # Left wheel reverse
                    wheel_speeds.angular.z = forward   # Right wheel forward
            else:  # CW turn
                if linear_x > self.linear_threshold:  # Forward + right turn
                    wheel_speeds.linear.x = forward    # Left wheel forward
                    wheel_speeds.angular.z = neutral   # Right wheel stop
                elif linear_x < -self.linear_threshold:  # Reverse + right turn
                    wheel_speeds.linear.x = neutral    # Left wheel stop
                    wheel_speeds.angular.z = reverse   # Right wheel reverse
                else:  # Spot turn CW
                    wheel_speeds.linear.x = forward    # Left wheel forward
                    wheel_speeds.angular.z = reverse   # Right wheel reverse
        else:  # Straight motion
            if linear_x > self.linear_threshold:  # Forward
                wheel_speeds.linear.x = forward    # Left wheel forward
                wheel_speeds.angular.z = forward   # Right wheel forward
            elif linear_x < -self.linear_threshold:  # Reverse
                wheel_speeds.linear.x = reverse    # Left wheel reverse
                wheel_speeds.angular.z = reverse   # Right wheel reverse
            else:  # Stop
                wheel_speeds.linear.x = neutral
                wheel_speeds.angular.z = neutral

        # Debug logging
        self.get_logger().info(
            f'CMD_VEL Input:\n'
            f'  Linear: {linear_x:6.3f} m/s, Angular: {angular_z:6.3f} rad/s\n'
            f'Wheel Speeds:\n'
            f'  Left PWM: {wheel_speeds.linear.x:.4f}\n'
            f'  Right PWM: {wheel_speeds.angular.z:.4f}'
        )
        
        self.wheel_speeds_pub.publish(wheel_speeds)

    def map_speed(self, speed_percent: float) -> float:
        """Convert speed percentage to PWM value using full min-max range"""
        if abs(speed_percent) < 0.5:
            return self.get_parameter('neutral_duty').value
            
        normalized = abs(speed_percent) / 100.0
        neutral = self.get_parameter('neutral_duty').value
        
        if speed_percent > 0:  # Forward
            # Forward: go from neutral to forward_max
            return neutral + normalized * (self.get_parameter('forward_max_duty').value - neutral)
        else:  # Reverse
            # Reverse: go from neutral to reverse_max
            return neutral - normalized * (neutral - self.get_parameter('reverse_max_duty').value)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigationController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
