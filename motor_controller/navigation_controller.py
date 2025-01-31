#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)  # Half of 0.32m
        self.declare_parameter('safety_margin', 0.1)  # Additional safety distance
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Minimum safe distance (robot radius + safety margin)
        self.min_distance = self.robot_radius + self.safety_margin
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Store latest scan data
        self.latest_scan = None
        
        self.get_logger().info('Navigation controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg

    def check_obstacles(self, desired_linear, desired_angular):
        """Check for obstacles and modify commands if needed"""
        if not self.latest_scan:
            return desired_linear, desired_angular
            
        # Get scan data as numpy array
        ranges = np.array(self.latest_scan.ranges)
        
        # Replace inf/nan with max range
        ranges[~np.isfinite(ranges)] = self.latest_scan.range_max
        
        # Calculate number of points in scan
        num_points = len(ranges)
        
        # Get angles for each scan point
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            num_points
        )
        
        # Check front area when moving forward
        if desired_linear > 0:
            # Look at points in front of robot (-30° to 30°)
            front_mask = np.abs(angles) < np.pi/6  # 30 degrees
            front_distances = ranges[front_mask]
            
            # If any point is too close, stop forward motion
            if np.any(front_distances < self.min_distance):
                min_front_dist = np.min(front_distances)
                self.get_logger().warn(
                    f'Obstacle detected at {min_front_dist:.2f}m, stopping forward motion'
                )
                desired_linear = 0.0
                
                # If very close, back up slightly
                if min_front_dist < self.robot_radius:
                    desired_linear = -0.1
        
        # Check sides when turning
        if abs(desired_angular) > 0:
            # Check left side when turning left
            if desired_angular > 0:
                left_mask = (angles > np.pi/4) & (angles < np.pi/2)  # 45° to 90°
                side_distances = ranges[left_mask]
            # Check right side when turning right
            else:
                right_mask = (angles < -np.pi/4) & (angles > -np.pi/2)  # -45° to -90°
                side_distances = ranges[right_mask]
            
            # If side obstacle is too close, reduce turning speed
            if len(side_distances) > 0 and np.any(side_distances < self.min_distance):
                min_side_dist = np.min(side_distances)
                self.get_logger().warn(
                    f'Side obstacle detected at {min_side_dist:.2f}m, reducing turn speed'
                )
                # Scale turn speed based on distance
                scale = min_side_dist / self.min_distance
                desired_angular *= max(0.2, scale)  # Minimum 20% of original turn speed
        
        # Ensure minimum angular speed when turning
        if abs(desired_angular) > 0:
            min_angular = 0.1  # Minimum 0.1 rad/s
            if abs(desired_angular) < min_angular:
                desired_angular = math.copysign(min_angular, desired_angular)
        
        return desired_linear, desired_angular

    def smooth_velocity(self, desired_linear, desired_angular):
        """Apply smoothing to velocity commands"""
        if not hasattr(self, 'last_linear'):
            self.last_linear = 0.0
            self.last_angular = 0.0
        
        # Smoothing factors (adjust these to change responsiveness)
        LINEAR_SMOOTH = 0.3   # Lower = smoother
        ANGULAR_SMOOTH = 0.2  # Lower = smoother
        
        # Apply exponential smoothing
        smooth_linear = (LINEAR_SMOOTH * desired_linear + 
                        (1 - LINEAR_SMOOTH) * self.last_linear)
        smooth_angular = (ANGULAR_SMOOTH * desired_angular + 
                         (1 - ANGULAR_SMOOTH) * self.last_angular)
        
        # Store for next iteration
        self.last_linear = smooth_linear
        self.last_angular = smooth_angular
        
        return smooth_linear, smooth_angular

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands with obstacle avoidance"""
        try:
            # Get desired speeds
            desired_linear = msg.linear.x
            desired_angular = msg.angular.z
            
            # Check for obstacles and modify commands if needed
            safe_linear, safe_angular = self.check_obstacles(desired_linear, desired_angular)
            
            # Create and publish wheel speeds message
            wheel_speeds = Twist()
            wheel_speeds.linear.x = safe_linear
            wheel_speeds.angular.z = safe_angular
            self.wheel_speeds_pub.publish(wheel_speeds)
            
            # Log if speeds were modified
            if smooth_linear != desired_linear or smooth_angular != desired_angular:
                self.get_logger().info(
                    f'Modified speeds for safety:\n'
                    f'  Linear: {desired_linear:.2f} -> {smooth_linear:.2f}\n'
                    f'  Angular: {desired_angular:.2f} -> {smooth_angular:.2f}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {str(e)}')
            # Stop robot on error
            stop_msg = Twist()
            self.wheel_speeds_pub.publish(stop_msg)

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
