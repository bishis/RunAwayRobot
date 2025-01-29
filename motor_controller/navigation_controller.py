#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .processors.path_planner import PathPlanner
from .processors.waypoint_generator import WaypointGenerator
import math
import numpy as np

class SimpleNavigationController(Node):
    def __init__(self):
        super().__init__('simple_navigation_controller')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('path_simplification_tolerance', 0.15)  # More precise path following
        self.declare_parameter('goal_tolerance', 0.12)   # Tighter positioning
        self.declare_parameter('angular_tolerance', 0.15) # More precise turning
        self.declare_parameter('max_linear_speed', 0.08)  # Slower for better control
        self.declare_parameter('max_angular_speed', 0.5) # Faster turning for tight spaces
        self.declare_parameter('min_segment_length', 0.2) # Even shorter segments for better precision
        
        # Robot physical parameters
        self.declare_parameter('wheel_separation', 0.24)
        self.declare_parameter('speed_exponent', 2.0)
        
        # Path handling
        self.current_path = None
        self.simplified_path = None
        self.current_segment = 0
        self.path_planner = PathPlanner(min_segment_length=self.get_parameter('min_segment_length').value)
        
        # Publishers/Subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, 10)
        self.simplified_path_pub = self.create_publisher(Path, '/simple_path', 10)  # For visualization
        
        # Control loop at 20Hz for more frequent path updates
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Control loop initialized at 20Hz for smoother path updates')
        
    def path_callback(self, msg: Path):
        """Receive and simplify new path"""
        self.get_logger().info('Received path message')
        if not msg.poses:
            self.get_logger().warning('Received empty path message')
            return
            
        self.current_path = msg
        original_length = len(msg.poses)
        self.get_logger().info(f'Original path length: {original_length}')
        
        # Validate and simplify path
        self.simplified_path = self.simplify_path(msg.poses)
        if not self.simplified_path:
            self.get_logger().error('Path simplification failed')
            return
            
        self.current_segment = 0
        simplified_length = len(self.simplified_path)
        
        # Log path simplification results
        self.get_logger().info(
            f'Path simplified from {original_length} to {simplified_length} points\n'
            f'Reduction ratio: {simplified_length/original_length:.2%}'
        )
        
        # Log path segments for debugging
        for i, pose in enumerate(self.simplified_path):
            self.get_logger().info(
                f'Segment {i}: x={pose.pose.position.x:.3f}, '
                f'y={pose.pose.position.y:.3f}'
            )
        
        # Publish simplified path for visualization with enhanced message construction
        simplified_path_msg = Path()
        simplified_path_msg.header.frame_id = 'map'  # Set the frame explicitly
        simplified_path_msg.header.stamp = self.get_clock().now().to_msg()  # Set current timestamp
        
        # Ensure proper pose stamping for visualization
        for pose in self.simplified_path:
            stamped_pose = PoseStamped()
            stamped_pose.header.frame_id = 'map'
            stamped_pose.header.stamp = self.get_clock().now().to_msg()
            stamped_pose.pose = pose.pose
            simplified_path_msg.poses.append(stamped_pose)
        
        # Publish with debug information
        self.simplified_path_pub.publish(simplified_path_msg)
        self.get_logger().info(
            f'Published simplified path for visualization:\n'
            f'  Frame ID: {simplified_path_msg.header.frame_id}\n'
            f'  Poses: {len(simplified_path_msg.poses)}\n'
            f'  First pose: ({simplified_path_msg.poses[0].pose.position.x:.3f}, '
            f'{simplified_path_msg.poses[0].pose.position.y:.3f})'
        )
        
    def simplify_path(self, poses):
        """Use PathPlanner to simplify the path"""
        if len(poses) < 2:
            self.get_logger().warning('Path too short to simplify')
            return poses
            
        try:
            return self.path_planner.simplify_path(poses)
        except Exception as e:
            self.get_logger().error(f'Error in path simplification: {str(e)}')
            return poses
        
    def get_robot_pose(self):
        """Get current robot pose in map frame"""
        try:
            # Add timeout for transform lookup
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            if not transform:
                self.get_logger().error('Received null transform')
                return None
            return transform
        except TransformException as ex:
            self.get_logger().error(f'Transform error: {ex}')
            return None
            
    def control_loop(self):
        """Enhanced control loop with forward-biased movement strategy and improved visualization"""
        if not self.simplified_path or self.current_segment >= len(self.simplified_path):
            self.get_logger().info('No path available or reached end of path')
            self.stop_robot()
            return

        self.get_logger().info(f'Path Progress: Segment {self.current_segment + 1}/{len(self.simplified_path)}')
            
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return
            
        target = self.simplified_path[self.current_segment]
        
        # Calculate distance and angle to target with enhanced logging
        dx = target.pose.position.x - robot_pose.transform.translation.x
        dy = target.pose.position.y - robot_pose.transform.translation.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Get robot's current heading with detailed position logging
        _, _, yaw = self.euler_from_quaternion(robot_pose.transform.rotation)
        angle_diff = self.normalize_angle(target_angle - yaw)
        
        # Log detailed position information
        self.get_logger().info(
            f' ahmed Navigation Status:\n'
            f'  Robot Position: ({robot_pose.transform.translation.x:.3f}, {robot_pose.transform.translation.y:.3f})\n'
            f'  Target Position: ({target.pose.position.x:.3f}, {target.pose.position.y:.3f})\n'
            f'  Distance to Target: {distance:.3f}m\n'
            f'  Heading Error: {math.degrees(angle_diff):.1f}°'
        )
        
        cmd = Twist()
        
        # Forward-biased movement strategy with state logging
        angular_tolerance = self.get_parameter('angular_tolerance').value
        max_angular_speed = self.get_parameter('max_angular_speed').value
        max_linear_speed = self.get_parameter('max_linear_speed').value
        
        # Movement state determination with logging
        movement_state = ''
        if abs(angle_diff) > math.pi/3:
            movement_state = 'Rotating in place'
            cmd.angular.z = max_angular_speed * 0.8 * math.copysign(1, angle_diff)
            cmd.linear.x = 0.0
        elif abs(angle_diff) > math.pi/8:
            movement_state = 'Combined turning and forward motion'
            turn_strength = min(1.0, abs(angle_diff) / (math.pi/4))
            cmd.angular.z = max_angular_speed * 0.6 * turn_strength * math.copysign(1, angle_diff)
            cmd.linear.x = max_linear_speed * (1 - turn_strength * 0.8)
        else:
            movement_state = 'Forward motion with course correction'
            forward_factor = math.cos(angle_diff)**2
            cmd.linear.x = max_linear_speed * forward_factor
            turn_factor = math.sin(angle_diff) * 0.8
            cmd.angular.z = max_angular_speed * turn_factor
            
            if abs(angle_diff) > angular_tolerance:
                cmd.angular.z = max_angular_speed * 0.4 * math.copysign(1, angle_diff)
                cmd.linear.x *= 0.9
        
        # Log movement state and commands
        self.get_logger().info(
            f'Movement State: {movement_state}\n'
            f'Command Velocities:\n'
            f'  Linear: {cmd.linear.x:.3f} m/s\n'
            f'  Angular: {cmd.angular.z:.3f} rad/s'
        )
        
        # Convert and publish wheel speeds with enhanced logging
        wheel_speeds = self.convert_twist_to_wheel_speeds(cmd)
        self.wheel_speeds_pub.publish(wheel_speeds)
        
        # Update segment progress
        if distance < self.get_parameter('goal_tolerance').value:
            self.get_logger().info(f'Reached waypoint {self.current_segment + 1}, moving to next segment')
            self.current_segment += 1
    
    def convert_twist_to_wheel_speeds(self, cmd: Twist) -> Twist:
        """Convert Twist commands to wheel speeds"""
        linear_x = max(min(cmd.linear.x, self.get_parameter('max_linear_speed').value), 
                      -self.get_parameter('max_linear_speed').value)
        angular_z = max(min(cmd.angular.z, self.get_parameter('max_angular_speed').value), 
                       -self.get_parameter('max_angular_speed').value)

        # Log input values
        self.get_logger().info(
            f'Input Twist:\n'
            f'  linear_x: {linear_x:.4f}\n'
            f'  angular_z: {angular_z:.4f}'
        )

        # Convert to wheel velocities
        wheel_separation = self.get_parameter('wheel_separation').value
        left_speed = linear_x - (angular_z * wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * wheel_separation / 2.0)

        # Log wheel velocities
        self.get_logger().info(
            f'Wheel Velocities:\n'
            f'  left: {left_speed:.4f}\n'
            f'  right: {right_speed:.4f}'
        )

        # Convert to PWM values
        wheel_speeds = Twist()
        wheel_speeds.linear.x = self.map_speed_to_pwm(left_speed)
        wheel_speeds.angular.z = self.map_speed_to_pwm(right_speed)

        # Log final PWM values
        self.get_logger().info(
            f'PWM Values:\n'
            f'  left: {wheel_speeds.linear.x:.4f}\n'
            f'  right: {wheel_speeds.angular.z:.4f}'
        )
        
        return wheel_speeds
    
    def map_speed_to_pwm(self, speed: float) -> float:
        """Convert speed to PWM value"""
        if abs(speed) < 0.02:  # Small dead zone
            return 0.075  # Neutral PWM
            
        # Normalize speed to percentage
        max_speed = self.get_parameter('max_linear_speed').value
        speed_percent = (speed / max_speed) * 100.0
        
        # Apply non-linear scaling
        normalized = abs(speed_percent) / 100.0
        
        if speed > 0:  # Forward
            return 0.075 + normalized * (0.10 - 0.075)  # Map to forward range
        else:  # Reverse
            return 0.075 - normalized * (0.075 - 0.045)  # Map to reverse range

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
        self.wheel_speeds_pub.publish(cmd)
        
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_between_poses(self, pose1, pose2):
        """Calculate distance between two poses"""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
        
    @staticmethod
    def euler_from_quaternion(q):
        """Convert quaternion to euler angles"""
        # Extract the values from q
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # Calculate roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Calculate pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Calculate yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main():
    rclpy.init()
    node = SimpleNavigationController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
