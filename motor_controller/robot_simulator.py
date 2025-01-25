#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update = time.time()
        
        # Robot parameters
        self.declare_parameter('wheel_separation', 0.24)  # meters
        self.declare_parameter('max_linear_speed', 0.1)   # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        
        # LIDAR parameters
        self.declare_parameter('laser_range', 5.0)        # meters
        self.declare_parameter('laser_samples', 360)      # number of samples
        self.declare_parameter('laser_noise', 0.01)       # meters of noise
        
        # Get parameters
        self.laser_range = self.get_parameter('laser_range').value
        self.laser_samples = self.get_parameter('laser_samples').value
        self.laser_noise = self.get_parameter('laser_noise').value
        
        # Subscribe to wheel speeds
        self.subscription = self.create_subscription(
            Twist,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize current speeds
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Define a more complex room environment for mapping
        self.walls = [
            # Outer boundary (10m x 8m room)
            (-5.0, -4.0, 5.0, -4.0),  # Bottom wall
            (5.0, -4.0, 5.0, 4.0),    # Right wall
            (5.0, 4.0, -5.0, 4.0),    # Top wall
            (-5.0, 4.0, -5.0, -4.0),  # Left wall

            # Room 1 - Bottom Left
            (-3.0, -4.0, -3.0, -1.0),  # Vertical wall
            (-3.0, -1.0, -1.0, -1.0),  # Horizontal wall
            
            # Room 2 - Bottom Right
            (2.0, -4.0, 2.0, -2.0),    # Vertical wall
            (2.0, -2.0, 4.0, -2.0),    # Horizontal wall
            
            # Central Area
            (-1.0, 0.0, 1.0, 0.0),     # Central divider
            (0.0, 0.0, 0.0, 2.0),      # T-junction
            
            # Room 3 - Top Left
            (-5.0, 2.0, -2.0, 2.0),    # Horizontal wall
            (-2.0, 2.0, -2.0, 4.0),    # Vertical wall
            
            # Room 4 - Top Right
            (2.0, 1.0, 2.0, 4.0),      # Vertical wall
            (2.0, 1.0, 4.0, 1.0),      # Horizontal wall
            
            # Obstacles
            (-4.0, -3.0, -3.5, -3.0),  # Small obstacle in Room 1
            (3.0, -3.0, 3.5, -3.0),    # Small obstacle in Room 2
            (-4.0, 3.0, -3.5, 3.0),    # Small obstacle in Room 3
            (3.0, 2.0, 3.5, 2.0),      # Small obstacle in Room 4
            
            # Doorways are created by gaps in the walls
        ]

        # Start position (in an open area)
        self.x = -4.0
        self.y = -3.0
        self.theta = 0.0
        
        # Update timer (50Hz)
        self.create_timer(0.02, self.update_pose)
        # LIDAR update timer (10Hz)
        self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('Robot simulator initialized')

    def wheel_speeds_callback(self, msg: Twist):
        """Store the current wheel speeds"""
        self.current_linear = msg.linear.x * self.get_parameter('max_linear_speed').value
        self.current_angular = -msg.angular.z * self.get_parameter('max_angular_speed').value

    def ray_intersection(self, ray_start, ray_angle, wall):
        """Calculate intersection of a ray with a wall segment"""
        x1, y1, x2, y2 = wall
        x3, y3 = ray_start
        x4 = x3 + math.cos(ray_angle) * self.laser_range
        y4 = y3 + math.sin(ray_angle) * self.laser_range
        
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if den == 0:  # parallel lines
            return None
            
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
        
        if 0 <= t <= 1 and u >= 0:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            distance = math.sqrt((x - x3)**2 + (y - y3)**2)
            return distance
        return None

    def publish_scan(self):
        """Publish simulated LIDAR scan"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = 2 * math.pi / self.laser_samples
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = self.laser_range
        
        ranges = []
        for i in range(self.laser_samples):
            angle = i * scan.angle_increment + self.theta
            ray_start = (self.x, self.y)
            
            # Find closest intersection with any wall
            min_distance = float('inf')
            for wall in self.walls:
                distance = self.ray_intersection(ray_start, angle, wall)
                if distance is not None and distance < min_distance:
                    min_distance = distance
            
            if min_distance == float('inf'):
                ranges.append(float('inf'))
            else:
                # Add some Gaussian noise
                noise = np.random.normal(0, self.laser_noise)
                ranges.append(min_distance + noise)
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)
        
        # Publish laser transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.z = 0.18  # Laser height
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def update_pose(self):
        """Update robot pose based on current speeds"""
        current_time = time.time()
        dt = current_time - self.last_update
        
        # Update pose
        if abs(self.current_angular) < 0.0001:  # Straight line motion
            self.x += self.current_linear * math.cos(self.theta) * dt
            self.y += self.current_linear * math.sin(self.theta) * dt
        else:  # Arc motion
            self.theta += self.current_angular * dt
            self.x += self.current_linear * math.cos(self.theta) * dt
            self.y += self.current_linear * math.sin(self.theta) * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Simple collision detection with walls
        robot_radius = 0.15  # meters
        for wall in self.walls:
            x1, y1, x2, y2 = wall
            # Calculate distance from robot to wall segment
            # (simplified - just checking endpoints for now)
            d1 = math.sqrt((self.x - x1)**2 + (self.y - y1)**2)
            d2 = math.sqrt((self.x - x2)**2 + (self.y - y2)**2)
            if d1 < robot_radius or d2 < robot_radius:
                # Collision detected - stop the robot
                self.current_linear = 0.0
                self.current_angular = 0.0
                break
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta/2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta/2.0)
        
        # Set velocity
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.angular.z = self.current_angular
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta/2.0)
        t.transform.rotation.w = math.cos(self.theta/2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        self.last_update = current_time

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 