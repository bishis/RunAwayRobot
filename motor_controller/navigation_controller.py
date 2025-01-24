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
        self.declare_parameter('path_simplification_tolerance', 0.2)  # Less aggressive simplification
        self.declare_parameter('goal_tolerance', 0.15)   # More precise positioning
        self.declare_parameter('angular_tolerance', 0.2) # More precise turning
        self.declare_parameter('max_linear_speed', 0.1)  # Keep normal speed
        self.declare_parameter('max_angular_speed', 0.4) # Slightly slower turning for better control
        self.declare_parameter('min_segment_length', 0.25) # Shorter segments for tight spaces
        
        # Path handling
        self.current_path = None
        self.simplified_path = None
        self.current_segment = 0
        
        # Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_publish', 10)
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, 10)
        
        # Control loop at 10Hz
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
        """Aggressively simplify path into major segments"""
        if len(poses) < 2:
            return poses
            
        # Start with first pose
        simplified = [poses[0]]
        min_segment_length = self.get_parameter('min_segment_length').value
        
        # Find significant direction changes
        current_direction = None
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            new_direction = math.atan2(dy, dx)
            
            # Add point if direction changes significantly or distance is large
            if current_direction is None or \
               abs(self.normalize_angle(new_direction - current_direction)) > math.pi/4 or \
               self.distance_between_poses(simplified[-1], poses[i]) > min_segment_length:
                simplified.append(poses[i])
                current_direction = new_direction
        
        # Always include last pose
        if simplified[-1] != poses[-1]:
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
        """Enhanced control loop with forward-biased movement strategy"""
        if not self.simplified_path or self.current_segment >= len(self.simplified_path):
            self.stop_robot()
            return
            
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return
            
        target = self.simplified_path[self.current_segment]
        
        # Calculate distance and angle to target
        dx = target.pose.position.x - robot_pose.transform.translation.x
        dy = target.pose.position.y - robot_pose.transform.translation.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Get robot's current heading
        _, _, yaw = self.euler_from_quaternion(robot_pose.transform.rotation)
        angle_diff = self.normalize_angle(target_angle - yaw)
        
        cmd = Twist()
        
        # Forward-biased movement strategy
        angular_tolerance = self.get_parameter('angular_tolerance').value
        max_angular_speed = self.get_parameter('max_angular_speed').value
        max_linear_speed = self.get_parameter('max_linear_speed').value
        
        # Improved turning strategy with hysteresis and smoother transitions
        if abs(angle_diff) > math.pi/3:  # More conservative threshold for spot turns
            # Pure rotation mode with gentle start/stop
            cmd.angular.z = max_angular_speed * 0.8 * math.copysign(1, angle_diff)
            cmd.linear.x = 0.0
        elif abs(angle_diff) > math.pi/8:  # Wider angle for combined motion
            # Balanced forward motion with coordinated turning
            turn_strength = min(1.0, abs(angle_diff) / (math.pi/4))
            cmd.angular.z = max_angular_speed * 0.6 * turn_strength * math.copysign(1, angle_diff)
            cmd.linear.x = max_linear_speed * (1 - turn_strength * 0.8)  # Gradual speed reduction
        else:
            # Forward-biased movement with gentle course corrections
            forward_factor = math.cos(angle_diff)**2  # Smoother speed transition
            cmd.linear.x = max_linear_speed * forward_factor
            
            # Proportional turning with reduced sensitivity
            turn_factor = math.sin(angle_diff) * 0.8  # Less aggressive turning
            cmd.angular.z = max_angular_speed * turn_factor
            
            # Minimal course corrections near target heading
            if abs(angle_diff) > angular_tolerance:
                cmd.angular.z = max_angular_speed * 0.4 * math.copysign(1, angle_diff)
                cmd.linear.x *= 0.9  # Very gentle speed reduction
        
        self.cmd_vel_pub.publish(cmd)
        
        # Update segment if close enough
        if distance < self.get_parameter('goal_tolerance').value:
            self.current_segment += 1

    def get_lookahead_point(self, robot_pose):
        """Find a point ahead on the path for smoother following"""
        lookahead = self.get_parameter('lookahead_distance').value
        
        # Start from current segment
        cumulative_dist = 0.0
        current_pos = robot_pose.transform.translation
        
        for i in range(self.current_segment, len(self.simplified_path)):
            target = self.simplified_path[i].pose.position
            segment_dist = math.sqrt(
                (target.x - current_pos.x)**2 + 
                (target.y - current_pos.y)**2
            )
            
            if cumulative_dist + segment_dist >= lookahead:
                # Interpolate to get exact lookahead point
                ratio = (lookahead - cumulative_dist) / segment_dist
                return type('Point', (), {
                    'x': current_pos.x + ratio * (target.x - current_pos.x),
                    'y': current_pos.y + ratio * (target.y - current_pos.y)
                })
            
            cumulative_dist += segment_dist
            current_pos = target
        
        # If we can't look ahead far enough, return the last point
        return self.simplified_path[-1].pose.position

    def distance_to_point(self, robot_pose, point):
        """Calculate distance from robot to a point"""
        dx = point.x - robot_pose.transform.translation.x
        dy = point.y - robot_pose.transform.translation.y
        return math.sqrt(dx*dx + dy*dy)
        
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
