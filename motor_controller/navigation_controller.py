#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('safety_radius', 0.3)
        self.declare_parameter('detection_distance', 0.5)
        self.declare_parameter('leg_length', 2.0)
        
        self.safety_radius = self.get_parameter('safety_radius').value
        self.detection_distance = self.get_parameter('detection_distance').value
        self.leg_length = self.get_parameter('leg_length').value
        
        # Navigation state
        self.current_leg = 0
        self.is_turning = False
        self.target_yaw = 0.0
        self.start_position = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_rf2o',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Control loop timer
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigation controller initialized')
        
        # Store latest data
        self.current_pose = None
        self.latest_scan = None
    
    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
        
    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg
        
    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion."""
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def check_obstacles(self):
        """Check for obstacles in path."""
        if not self.latest_scan:
            return True
            
        # Check front area (assuming LIDAR 0° is front)
        front_angles = range(-30, 31)  # -30° to +30°
        for angle in front_angles:
            idx = (angle + 360) % 360
            if 0 < self.latest_scan.ranges[idx] < self.safety_radius:
                return True
        return False
    
    def control_loop(self):
        """Main control loop."""
        if not self.current_pose or not self.latest_scan:
            self.get_logger().warn('Waiting for sensor data...')
            return
            
        # Get current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Initialize start position if needed
        if self.start_position is None:
            self.start_position = (current_x, current_y)
            self.get_logger().info(f'Start position set to: ({current_x:.2f}, {current_y:.2f})')
        
        # Check for obstacles
        if self.check_obstacles():
            self.get_logger().warn('Obstacle detected!')
            self.stop_robot()
            return
        
        # Calculate distance traveled in current leg
        if self.current_leg % 2 == 0:  # Even legs (forward)
            distance = abs(current_y - self.start_position[1])
        else:  # Odd legs (sideways)
            distance = abs(current_x - self.start_position[0])
        
        # Debug info
        self.get_logger().info(
            f'Position: ({current_x:.2f}, {current_y:.2f}), '
            f'Yaw: {math.degrees(current_yaw):.1f}°, '
            f'Distance: {distance:.2f}m'
        )
        
        # Handle movement
        cmd = Twist()
        if distance >= self.leg_length:
            # Time to turn
            if not self.is_turning:
                self.is_turning = True
                self.target_yaw = (self.current_leg + 1) * 90
                if self.target_yaw >= 360:
                    self.target_yaw -= 360
            
            # Calculate turn
            angle_diff = self.target_yaw - math.degrees(current_yaw)
            while angle_diff > 180: angle_diff -= 360
            while angle_diff < -180: angle_diff += 360
            
            if abs(angle_diff) > 5:
                # Keep turning
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Turn complete
                self.is_turning = False
                self.current_leg += 1
                self.start_position = (current_x, current_y)
        else:
            # Move forward
            cmd.linear.x = 0.2
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 