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
        self.declare_parameter('forward_max_duty', 0.10)
        self.declare_parameter('forward_min_duty', 0.09)
        self.declare_parameter('reverse_max_duty', 0.045)
        self.declare_parameter('reverse_min_duty', 0.05)
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
        """Convert Twist commands to wheel speeds"""
        # Clamp input values
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # Convert to wheel velocities
        left_speed = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert to percentages
        left_percent = (left_speed / self.max_linear_speed) * 100.0
        right_percent = (right_speed / self.max_linear_speed) * 100.0

        # Enhanced spot turn handling
        if abs(angular_z) > self.angular_threshold:
            turn_power = min(100.0, abs(angular_z) / self.max_angular_speed * 100.0)
            
            if abs(linear_x) < self.linear_threshold:  # Pure spot turn
                if angular_z > 0:  # CCW turn
                    left_percent = -turn_power
                    right_percent = turn_power
                else:  # CW turn
                    left_percent = turn_power
                    right_percent = -turn_power
            else:  # Turning while moving
                turn_factor = turn_power / 100.0
                if angular_z > 0:  # CCW turn
                    left_percent *= (1.0 - turn_factor)
                    right_percent *= (1.0 + turn_factor)
                else:  # CW turn
                    left_percent *= (1.0 + turn_factor)
                    right_percent *= (1.0 - turn_factor)

        # Create wheel speeds message
        wheel_speeds = Twist()
        wheel_speeds.linear.x = self.map_speed(left_percent)   # Left wheel
        wheel_speeds.angular.z = self.map_speed(right_percent) # Right wheel
        
        # Publish wheel speeds
        self.wheel_speeds_pub.publish(wheel_speeds)
        
        # Debug logging
        self.get_logger().debug(
            f'Wheel Speeds:\n'
            f'  Left:  {left_percent:6.1f}% PWM: {wheel_speeds.linear.x:.4f}\n'
            f'  Right: {right_percent:6.1f}% PWM: {wheel_speeds.angular.z:.4f}\n'
            f'Command: Linear: {linear_x:6.3f} m/s Angular: {angular_z:6.3f} rad/s'
        )

    def map_speed(self, speed_percent: float) -> float:
        """Convert speed percentage to PWM value"""
        if abs(speed_percent) < 0.5:
            return self.get_parameter('neutral_duty').value
            
        normalized = abs(speed_percent) / 100.0
        
        if speed_percent > 0:  # Forward
            return self.get_parameter('neutral_duty').value + \
                   normalized * (self.get_parameter('forward_max_duty').value - 
                               self.get_parameter('neutral_duty').value)
        else:  # Reverse
            return self.get_parameter('neutral_duty').value - \
                   normalized * (self.get_parameter('neutral_duty').value - 
                               self.get_parameter('reverse_max_duty').value)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigationController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
