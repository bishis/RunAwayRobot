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
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('path_simplification_tolerance', 0.1)  # meters
        self.declare_parameter('goal_tolerance', 0.1)  # meters
        self.declare_parameter('angular_tolerance', 0.1)  # radians
        self.declare_parameter('max_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        
        # Path handling
        self.current_path = None
        self.simplified_path = None
        self.current_segment = 0
        
        # Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, 10)
        
        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
    def path_callback(self, msg: Path):
        """Receive and simplify new path"""
        if not msg.poses:
            return
            
        self.current_path = msg
        self.simplified_path = self.simplify_path(msg.poses)
        self.current_segment = 0
        self.get_logger().info(f'Received new path with {len(self.simplified_path)} segments')
        
    def simplify_path(self, poses):
        """Convert path into straight-line segments"""
        if len(poses) < 2:
            return poses
            
        tolerance = self.get_parameter('path_simplification_tolerance').value
        simplified = [poses[0]]
        
        for i in range(1, len(poses)):
            # Check if we need a new segment
            if self.point_line_distance(poses[i].pose.position, 
                                     simplified[-1].pose.position,
                                     poses[i-1].pose.position) > tolerance:
                simplified.append(poses[i-1])
        
        simplified.append(poses[-1])
        return simplified
        
    def point_line_distance(self, point, line_start, line_end):
        """Calculate point-to-line distance"""
        if line_start == line_end:
            return math.sqrt(
                (point.x - line_start.x)**2 + 
                (point.y - line_start.y)**2
            )
            
        return abs(
            (line_end.y - line_start.y) * point.x -
            (line_end.x - line_start.x) * point.y +
            line_end.x * line_start.y -
            line_end.y * line_start.x
        ) / math.sqrt(
            (line_end.y - line_start.y)**2 +
            (line_end.x - line_start.x)**2
        )
        
    def get_robot_pose(self):
        """Get current robot pose in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            return transform
        except TransformException as ex:
            self.get_logger().warning(f'Could not get transform: {ex}')
            return None
            
    def control_loop(self):
        """Main control loop for path following"""
        if not self.simplified_path or self.current_segment >= len(self.simplified_path):
            self.stop_robot()
            return
            
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return
            
        # Get current segment target
        target = self.simplified_path[self.current_segment]
        
        # Calculate distance and angle to target
        dx = target.pose.position.x - robot_pose.transform.translation.x
        dy = target.pose.position.y - robot_pose.transform.translation.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Get robot's current heading
        _, _, yaw = self.euler_from_quaternion(robot_pose.transform.rotation)
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(target_angle - yaw)
        
        # Generate control commands
        cmd = Twist()
        
        # First align with the target
        if abs(angle_diff) > self.get_parameter('angular_tolerance').value:
            cmd.angular.z = self.get_parameter('max_angular_speed').value * (angle_diff / math.pi)
        # Then move towards it
        else:
            cmd.linear.x = self.get_parameter('max_linear_speed').value * min(1.0, distance)
            cmd.angular.z = self.get_parameter('max_angular_speed').value * (angle_diff / math.pi)
        
        # Check if we've reached the current segment target
        if distance < self.get_parameter('goal_tolerance').value:
            self.current_segment += 1
            
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    @staticmethod
    def euler_from_quaternion(q):
        """Convert quaternion to euler angles"""
        # Implementation of quaternion to euler conversion
        # You can use existing implementations or tf2 functions
        pass

def main():
    rclpy.init()
    node = SimpleNavigationController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
