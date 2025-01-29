#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from tf2_ros import TransformListener, Buffer

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)
        self.declare_parameter('safety_margin', 0.15)  # Increased safety margin
        self.declare_parameter('detection_radius', 0.5)  # Radius to check for obstacles
        self.declare_parameter('replanning_radius', 0.8)  # Radius to trigger replanning
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.detection_radius = self.get_parameter('detection_radius').value
        self.replanning_radius = self.get_parameter('replanning_radius').value
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Publisher for detected obstacles (to update map)
        self.obstacle_pub = self.create_publisher(OccupancyGrid, 'detected_obstacles', 10)
        
        # Store latest data
        self.latest_scan = None
        self.last_obstacle_check = self.get_clock().now()
        self.obstacle_check_period = 0.2  # Check every 200ms

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg

    def check_obstacles(self, desired_linear, desired_angular):
        """Enhanced obstacle checking with detection zones"""
        if not self.latest_scan:
            return desired_linear, desired_angular, False
            
        now = self.get_clock().now()
        new_obstacle_detected = False
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            len(ranges)
        )
        
        # Create detection zones
        immediate_zone = self.robot_radius + self.safety_margin
        caution_zone = self.detection_radius
        planning_zone = self.replanning_radius
        
        # Check zones in movement direction
        if desired_linear > 0:
            front_mask = np.abs(angles) < np.pi/4  # 45 degrees
            front_distances = ranges[front_mask]
            front_angles = angles[front_mask]
            
            # Immediate stop zone
            if np.any(front_distances < immediate_zone):
                desired_linear = 0.0
            
            # Caution zone - slow down
            elif np.any(front_distances < caution_zone):
                desired_linear *= 0.5
            
            # Planning zone - check for new obstacles
            if (now - self.last_obstacle_check).nanoseconds / 1e9 > self.obstacle_check_period:
                planning_mask = front_distances < planning_zone
                if np.any(planning_mask):
                    # Get obstacle positions
                    obstacle_angles = front_angles[planning_mask]
                    obstacle_ranges = front_distances[planning_mask]
                    new_obstacle_detected = True
                    self.publish_obstacle_update(obstacle_ranges, obstacle_angles)
                    self.last_obstacle_check = now
        
        # Similar checks for reverse motion
        elif desired_linear < 0:
            rear_mask = np.abs(np.abs(angles) - np.pi) < np.pi/4
            rear_distances = ranges[rear_mask]
            
            if np.any(rear_distances < immediate_zone):
                desired_linear = 0.0
            elif np.any(rear_distances < caution_zone):
                desired_linear *= 0.5
        
        # Check sides when turning
        if abs(desired_angular) > 0:
            if desired_angular > 0:  # Left turn
                side_mask = (angles > np.pi/6) & (angles < np.pi/2)
            else:  # Right turn
                side_mask = (angles < -np.pi/6) & (angles > -np.pi/2)
            
            side_distances = ranges[side_mask]
            if len(side_distances) > 0:
                min_side_dist = np.min(side_distances)
                if min_side_dist < immediate_zone:
                    desired_angular = 0.0
                elif min_side_dist < caution_zone:
                    desired_angular *= 0.5
        
        return desired_linear, desired_angular, new_obstacle_detected

    def publish_obstacle_update(self, ranges, angles):
        """Publish detected obstacles for map updates"""
        if not hasattr(self, 'tf_buffer'):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        
        try:
            # Get robot's position in map frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_yaw = 2 * math.acos(transform.transform.rotation.w)
            
            # Convert laser detections to map coordinates
            obstacle_cells = []
            for r, theta in zip(ranges, angles):
                # Convert polar to cartesian coordinates
                x = robot_x + r * math.cos(theta + robot_yaw)
                y = robot_y + r * math.sin(theta + robot_yaw)
                obstacle_cells.append((x, y))
            
            # Create and publish obstacle message
            obstacle_msg = OccupancyGrid()
            obstacle_msg.header.frame_id = 'map'
            obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            # Fill in obstacle data...
            self.obstacle_pub.publish(obstacle_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to publish obstacles: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands with obstacle avoidance"""
        try:
            # Get desired speeds
            desired_linear = msg.linear.x
            desired_angular = msg.angular.z
            
            # Check for obstacles and modify commands if needed
            safe_linear, safe_angular, new_obstacle_detected = self.check_obstacles(desired_linear, desired_angular)
            
            # Create and publish wheel speeds message
            wheel_speeds = Twist()
            wheel_speeds.linear.x = safe_linear
            wheel_speeds.angular.z = safe_angular
            self.wheel_speeds_pub.publish(wheel_speeds)
            
            # Log if speeds were modified
            if safe_linear != desired_linear or safe_angular != desired_angular:
                self.get_logger().info(
                    f'Modified speeds for safety:\n'
                    f'  Linear: {desired_linear:.2f} -> {safe_linear:.2f}\n'
                    f'  Angular: {desired_angular:.2f} -> {safe_angular:.2f}'
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
