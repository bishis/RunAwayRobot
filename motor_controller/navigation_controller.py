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
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('linear_threshold', 0.01)
        self.declare_parameter('angular_threshold', 0.02)
        self.declare_parameter('position_tolerance', 0.15)    # Increased position tolerance
        self.declare_parameter('angle_tolerance', 0.25)      # Increased angle tolerance (~15 degrees)
        
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
        
        # Initialize all state variables
        self.current_path = None
        self.current_commands = []
        self.current_command_index = 0
        self.current_target_position = None
        self.current_target_heading = None
        
        # Path update tracking
        self.path_update_pending = False
        self.latest_path = None
        
        # Movement control parameters
        self.last_turn_direction = None
        self.direction_change_time = None
        self.min_turn_duration = 1.5        # Longer minimum turn time
        self.turn_hysteresis = 0.45        # ~25 degrees hysteresis
        self.consecutive_turns = 0         # Count consecutive direction changes
        self.max_consecutive_turns = 2      # Reduced max turns before pause
        self.last_command_time = None      # Track command timing
        self.pause_duration = 1.0          # Pause duration after max turns
        self.last_pause_time = None        # Track pause timing
        self.forward_correction_gain = 0.3  # Reduced correction during forward movement
        
        # Add rotation control parameters
        self.rotation_start_time = None
        self.max_rotation_time = 5.0  # Maximum time to attempt a rotation (seconds)
        self.final_turn_hysteresis = 0.6  # Larger hysteresis for final turns (~35 degrees)
        self.initial_turn_direction = None  # Track initial turn direction
        
        self.get_logger().info('Navigation controller initialized')

    def path_callback(self, msg: Path):
        """Handle new path from planner"""
        if not msg.poses:
            return
            
        # Get current robot pose
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return
        
        # Get current heading
        current_heading = self._get_yaw_from_quaternion(robot_pose.rotation)
        
        # Store the new path and mark for update
        self.latest_path = msg
        self.path_update_pending = True
        
        self.get_logger().info('Received new path - will update current plan')

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
        # Check if we need to update to a new path
        if self.path_update_pending and self.latest_path:
            # Convert path poses to points
            waypoints = [pose.pose.position for pose in self.latest_path.poses]
            
            # Get current robot pose for new plan
            robot_pose = self.get_robot_pose()
            if robot_pose:
                current_heading = self._get_yaw_from_quaternion(robot_pose.rotation)
                
                # Generate new commands from current position
                self.current_commands = self.path_planner.simplify_path(
                    waypoints, 
                    initial_heading=current_heading
                )
                self.current_command_index = 0
                self.current_target_position = None
                self.current_target_heading = None
                self.last_turn_direction = None
                self.direction_change_time = None
                
                # Create and publish visualization markers
                markers = self.path_planner.create_visualization_markers(
                    self.current_commands,
                    waypoints[0],
                    frame_id='map'
                )
                self.marker_pub.publish(markers)
                
                self.get_logger().info(
                    f'Updated to new path with {len(self.current_commands)} commands. '
                    f'Current heading: {math.degrees(current_heading):.1f}°'
                )
            
            self.path_update_pending = False
            self.latest_path = None

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
            self.last_turn_direction = None  # Reset turn direction
            self.direction_change_time = None
            
            if cmd_type == 'rotate':
                self.current_target_heading = self._normalize_angle(current_heading + value)
                self.current_target_position = robot_pos
                self.get_logger().info(
                    f'Starting rotation: current={math.degrees(current_heading):.1f}°, '
                    f'target={math.degrees(self.current_target_heading):.1f}°'
                )
            else:  # forward
                self.current_target_heading = current_heading
                self.current_target_position = robot_pos + value * np.array([
                    math.cos(current_heading),
                    math.sin(current_heading)
                ])
                self.get_logger().info(f'Starting forward: distance={value:.2f}m')
        
        # Generate velocity command
        cmd = Twist()
        current_heading = self._normalize_angle(self._get_yaw_from_quaternion(robot_pose.rotation))
        current_time = self.get_clock().now()

        if cmd_type == 'rotate':
            # Handle rotation with improved smoothing
            angle_diff = self._normalize_angle(self.current_target_heading - current_heading)
            command_complete = abs(angle_diff) < self.angle_tolerance
            
            if not command_complete:
                # Initialize rotation start time if needed
                if self.rotation_start_time is None:
                    self.rotation_start_time = current_time
                    self.initial_turn_direction = -1.0 if angle_diff > 0 else 1.0
                
                # Check for rotation timeout
                elapsed_rotation = (current_time - self.rotation_start_time).nanoseconds / 1e9
                if elapsed_rotation > self.max_rotation_time:
                    self.get_logger().warning('Rotation timed out, marking as complete')
                    command_complete = True
                else:
                    # Check if we're in a pause state
                    if self.last_pause_time is not None:
                        elapsed = (current_time - self.last_pause_time).nanoseconds / 1e9
                        if elapsed < self.pause_duration:
                            cmd.angular.z = 0.0
                            return
                        self.last_pause_time = None
                        self.consecutive_turns = 0

                    # Determine turn direction with stronger hysteresis for final rotations
                    desired_direction = -1.0 if angle_diff > 0 else 1.0
                    
                    # Use initial direction if angle is within hysteresis
                    if abs(angle_diff) < self.final_turn_hysteresis:
                        desired_direction = self.initial_turn_direction
                    
                    # Update direction tracking
                    if self.last_turn_direction != desired_direction:
                        if self.last_turn_direction is not None:
                            self.consecutive_turns += 1
                            if self.consecutive_turns > self.max_consecutive_turns:
                                self.get_logger().info('Rotation unstable, using initial direction')
                                desired_direction = self.initial_turn_direction
                                self.consecutive_turns = 0
                        
                        self.last_turn_direction = desired_direction
                        self.direction_change_time = current_time
                    
                    cmd.angular.z = desired_direction
                    cmd.linear.x = 0.0
                    
                    self.get_logger().debug(
                        f'Rotation: diff={math.degrees(angle_diff):.1f}°, '
                        f'direction={"LEFT" if cmd.angular.z < 0 else "RIGHT"}, '
                        f'elapsed={elapsed_rotation:.1f}s'
                    )
            
            if command_complete:
                # Reset rotation control variables
                self.rotation_start_time = None
                self.initial_turn_direction = None
                
        else:  # forward
            # Calculate direction to target
            to_target = self.current_target_position - np.array([robot_pose.translation.x, robot_pose.translation.y])
            distance = math.sqrt(np.sum(to_target * to_target))
            command_complete = distance < self.position_tolerance
            
            if not command_complete:
                # Calculate heading error
                target_heading = math.atan2(to_target[1], to_target[0])
                heading_error = self._normalize_angle(target_heading - current_heading)
                
                # More forgiving forward movement with smoother corrections
                if abs(heading_error) < self.angle_tolerance * 2.0:  # Double tolerance for forward
                    cmd.linear.x = 1.0
                    # Proportional heading correction
                    if abs(heading_error) > self.angle_tolerance * 0.25:  # Start correction earlier
                        correction = self.forward_correction_gain * heading_error / self.angle_tolerance
                        cmd.angular.z = max(-0.5, min(0.5, -correction))  # Limit correction
                    else:
                        cmd.angular.z = 0.0
                else:
                    # Need to stop and correct heading
                    cmd.linear.x = 0.0
                    cmd.angular.z = -1.0 if heading_error > 0 else 1.0
                    self.get_logger().debug(f'Stopping to correct large heading error: {math.degrees(heading_error):.1f}°')

        # Enforce minimum command duration and smooth transitions
        if self.last_command_time is not None:
            elapsed = (current_time - self.last_command_time).nanoseconds / 1e9
            if elapsed < 0.2:  # Increased minimum duration between changes
                return
        self.last_command_time = current_time

        # Publish command
        self.wheel_speeds_pub.publish(cmd)
        
        # Reset consecutive turns counter when command completes
        if command_complete:
            self.consecutive_turns = 0
            self.get_logger().info(
                f'Completed {cmd_type} command: '
                f'heading={math.degrees(current_heading):.1f}°'
            )
            self.current_command_index += 1
            self.current_target_position = None
            self.current_target_heading = None
            self.last_turn_direction = None
            self.direction_change_time = None
            self.rotation_start_time = None
            self.initial_turn_direction = None
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

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

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
