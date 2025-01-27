#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from .processors.path_planner import PathPlanner
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
from visualization_msgs.msg import MarkerArray
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters for speed conversion
        self.declare_parameter('max_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('linear_threshold', 0.01)
        self.declare_parameter('angular_threshold', 0.02)
        self.declare_parameter('position_tolerance', 0.05)  # meters
        self.declare_parameter('angle_tolerance', 0.1)     # radians
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # Initialize path planner
        self.path_planner = PathPlanner(
            angle_threshold=self.angular_threshold,
            min_segment_length=self.linear_threshold
        )
        
        # Navigation state
        self.current_path = None
        self.current_commands = []
        self.current_command_index = 0
        self.command_start_time = None
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            'plan',
            self.path_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/path_planner_viz', 10)
        
        # Create timer for command execution
        self.create_timer(0.1, self.execute_commands)  # 10Hz control loop
        
        self.get_logger().info('Navigation controller initialized')
        
        # Add to state tracking
        self.current_target_position = None
        self.current_target_heading = None

    def path_callback(self, msg: Path):
        """Handle new path from planner"""
        if not msg.poses:
            return
            
        # Convert path poses to points
        waypoints = [pose.pose.position for pose in msg.poses]
        
        # Generate simplified commands
        self.current_commands = self.path_planner.simplify_path(waypoints)
        self.current_command_index = 0
        self.command_start_time = None
        
        # Create and publish visualization markers
        markers = self.path_planner.create_visualization_markers(
            self.current_commands,
            waypoints[0],
            frame_id='map'
        )
        self.marker_pub.publish(markers)
        
        self.get_logger().info(f'Received new path with {len(self.current_commands)} commands')

    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            return transform.transform
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose: {ex}')
            return None

    def execute_commands(self):
        """Execute current command with position feedback"""
        if not self.current_commands or self.current_command_index >= len(self.current_commands):
            return
            
        # Get current robot pose
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return
            
        # Get current command
        cmd_type, value = self.current_commands[self.current_command_index]
        
        # Initialize target position/heading if needed
        if self.current_target_position is None:
            robot_pos = np.array([robot_pose.translation.x, robot_pose.translation.y])
            current_heading = self._get_yaw_from_quaternion(robot_pose.rotation)
            
            if cmd_type == 'rotate':
                self.current_target_heading = current_heading + value
                self.current_target_position = robot_pos
            else:  # forward
                self.current_target_heading = current_heading
                self.current_target_position = robot_pos + value * np.array([
                    math.cos(current_heading),
                    math.sin(current_heading)
                ])
            
            self.get_logger().info(f'Starting {cmd_type}: target={value:.2f}')
        
        # Check if command is complete
        current_pos = np.array([robot_pose.translation.x, robot_pose.translation.y])
        current_heading = self._get_yaw_from_quaternion(robot_pose.rotation)
        
        command_complete = False
        if cmd_type == 'rotate':
            angle_diff = self._normalize_angle(
                self.current_target_heading - current_heading
            )
            command_complete = abs(angle_diff) < self.angle_tolerance
        else:  # forward
            pos_diff = self.current_target_position - current_pos
            distance = math.sqrt(np.sum(pos_diff * pos_diff))
            command_complete = distance < self.position_tolerance
        
        # Generate velocity command
        cmd = Twist()
        if cmd_type == 'rotate':
            angle_diff = self._normalize_angle(
                self.current_target_heading - current_heading
            )
            cmd.angular.z = 1.0 if angle_diff > 0 else -1.0
            cmd.linear.x = 0.0
        else:  # forward
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0
        
        # Publish command
        self.wheel_speeds_pub.publish(cmd)
        
        # Move to next command if complete
        if command_complete:
            self.current_command_index += 1
            self.current_target_position = None
            self.current_target_heading = None
            if self.current_command_index >= len(self.current_commands):
                self.get_logger().info('Path execution complete')
                self.wheel_speeds_pub.publish(Twist())

    def cmd_vel_callback(self, msg: Twist):
        """Handle direct velocity commands (override path following)"""
        # Create wheel speeds message
        wheel_speeds = Twist()
        
        # Convert linear velocity to speed channel value
        if abs(msg.linear.x) > self.linear_threshold:
            wheel_speeds.linear.x = 1.0 if msg.linear.x > 0 else -1.0
        else:
            wheel_speeds.linear.x = 0.0

        # Convert angular velocity to turn channel value
        if abs(msg.angular.z) > self.angular_threshold:
            wheel_speeds.angular.z = 1.0 if msg.angular.z < 0 else -1.0  # Right is positive
        else:
            wheel_speeds.angular.z = 0.0
        
        # Debug logging
        self.get_logger().info(
            f'CMD_VEL Input:\n'
            f'  Linear: {msg.linear.x:6.3f} m/s, Angular: {msg.angular.z:6.3f} rad/s\n'
            f'Binary Servo Output:\n'
            f'  Speed Channel: {wheel_speeds.linear.x:4.1f} {"(FWD)" if wheel_speeds.linear.x > 0 else "(REV)" if wheel_speeds.linear.x < 0 else "(STOP)"}\n'
            f'  Turn Channel: {wheel_speeds.angular.z:4.1f} {"(RIGHT)" if wheel_speeds.angular.z > 0 else "(LEFT)" if wheel_speeds.angular.z < 0 else "(CENTER)"}'
        )
        
        # Publish wheel speeds
        self.wheel_speeds_pub.publish(wheel_speeds)

    def _get_yaw_from_quaternion(self, q):
        """Extract yaw angle from quaternion"""
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_yaw_from_quaternion(self, q):
        """Extract yaw angle from quaternion"""
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
