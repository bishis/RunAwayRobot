#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import numpy as np
import math
from .processors.waypoint_generator import WaypointGenerator
from std_msgs.msg import Bool
from .processors.human_avoidance_controller import HumanAvoidanceController
from std_srvs.srv import Empty
from tf2_ros import TransformException, Buffer, TransformListener
import time
from builtin_interfaces.msg import Time

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Initialize tf2 buffer and listener FIRST
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Pass 'self' as the node
        
        # Initialize current_pose with proper structure
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.w = 1.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 1.0)  # Actual max rotation speed
        self.declare_parameter('min_rotation_speed', 0.8)
        self.declare_parameter('goal_timeout', 30.0)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_rotation_speed = self.get_parameter('min_rotation_speed').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        
        # Initialize waypoint generator AFTER tf setup
        self.waypoint_generator = WaypointGenerator(
            node=self,
            min_distance=0.5,
            safety_margin=self.safety_margin,
            waypoint_size=0.3,
            preferred_distance=1.0,
            goal_tolerance=0.3
        )
        
        # Add current_map storage
        self.current_map = None
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 10)
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Add debug logging for goal sending
        self.get_logger().info('Waiting for navigation action server...')
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Still waiting for navigation action server...')
        self.get_logger().info('Navigation server connected!')
        
        # State variables
        self.latest_scan = None
        self.current_goal = None
        self.is_navigating = False
        self.goal_start_time = None
        self.previous_waypoint = None

        # Add state for Nav2 readiness
        self.nav2_ready = False
        self.nav2_check_timer = self.create_timer(1.0, self.check_nav2_ready)
        
        # Create timer for exploration control every 0.1 seconds
        self.exploration_loop_timer = self.create_timer(0.1, self.exploration_loop)

        # Add timeout parameters
        self.goal_timeout = 15.0  # Shorter timeout for unreachable goals
        self.planning_attempts = 0
        self.max_planning_attempts = 2  # Max attempts before giving up
        
        # Add timer to check goal progress every 0.1 seconds
        self.goal_check_timer = self.create_timer(0.1, self.check_goal_progress)  
        
        # Add human tracking subscribers
        self.tracking_active_sub = self.create_subscription(
            Bool,
            '/human_tracking_active',
            self.tracking_active_callback,
            10
        )
        
        self.tracking_cmd_sub = self.create_subscription(
            Twist,
            '/human_tracking_cmd',
            self.tracking_cmd_callback,
            10
        )
        
        self.is_tracking_human = False
        
        self.get_logger().info('Navigation controller initialized')
        
        self._current_goal_handle = None

        self.human_avoidance = HumanAvoidanceController(self, self.waypoint_generator)

        # Add escape-specific parameters
        self.escape_timeout = 45.0  # Longer timeout for escape attempts
        self.max_escape_attempts = 3  # Number of retry attempts for escape
        self.escape_attempts = 0  # Counter for escape attempts
        
        # Add storage for last seen human position
        self.last_human_position = None
        self.last_human_timestamp = None

        # Add timer for escape monitoring (initially disabled)
        self.escape_monitor_timer = None
        self.escape_wait_start = None

        # Add map publisher for human obstacle updates
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        
        # Add spin parameters
        self.SPIN_SPEED = 0.8  # rad/s
        self.is_spinning = False
        self.spin_timer = None

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg
        
        # Update current_pose based on the latest transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            # Update current_pose with the latest transform
            self.current_pose.pose.position.x = transform.transform.translation.x
            self.current_pose.pose.position.y = transform.transform.translation.y
            self.current_pose.pose.position.z = transform.transform.translation.z
            self.current_pose.pose.orientation = transform.transform.rotation
        except TransformException:
            self.get_logger().warn('Could not get robot position from transform')
        
        # Pass scan to human avoidance controller
        if hasattr(self, 'human_avoidance'):
            self.human_avoidance.latest_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        """Update map in waypoint generator and store locally"""
        self.current_map = msg  # Store map locally
        self.waypoint_generator.update_map(msg)

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        try:
            # Simply pass through the commands
            wheel_speeds = Twist()
            wheel_speeds.linear.x = msg.linear.x
            wheel_speeds.angular.z = msg.angular.z
            self.wheel_speeds_pub.publish(wheel_speeds)
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {str(e)}')
            self.wheel_speeds_pub.publish(Twist())

    def check_nav2_ready(self):
        """Check if Nav2 stack is ready"""
        try:
            if not self.nav2_ready:
                if self.nav_client.wait_for_server(timeout_sec=0.1):
                    self.get_logger().info('Nav2 stack is ready!')
                    self.nav2_ready = True
                    # Stop checking once ready
                    self.nav2_check_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f'Error checking Nav2 readiness: {str(e)}')

    def exploration_loop(self):
        """Modified exploration loop to handle human tracking and map completion"""
        if self.is_tracking_human:
            return
            
        try:
            if not self.nav2_ready:
                return
            if not self.is_navigating:
                # Store current waypoint before generating new one
                self.previous_waypoint = self.current_goal
                
                waypoint = self.waypoint_generator.generate_waypoint()
                if waypoint:
                    # Check if waypoint is same as previous
                    if self.previous_waypoint and \
                       abs(waypoint.pose.position.x - self.previous_waypoint.pose.position.x) < 0.1 and \
                       abs(waypoint.pose.position.y - self.previous_waypoint.pose.position.y) < 0.1:
                        self.get_logger().warn('bishi Generated waypoint is too similar to previous, forcing new one')
                        self.waypoint_generator.force_waypoint_change()
                        return
                        
                    # Check if waypoint is near wall
                    if self.current_map and not self.waypoint_generator.is_near_wall(
                        waypoint.pose.position.x,
                        waypoint.pose.position.y,
                        np.array(self.current_map.data).reshape(
                            self.current_map.info.height,
                            self.current_map.info.width
                        ),
                        self.current_map.info.resolution,
                        self.current_map.info.origin.position.x,
                        self.current_map.info.origin.position.y
                    ):
                        self.current_goal = waypoint  # Store new goal
                        self.send_goal(waypoint)
                        # Green for exploration
                        markers = self.waypoint_generator.create_visualization_markers(waypoint, is_escape=False)
                        self.marker_pub.publish(markers)
                    else:
                        self.get_logger().warn('Generated waypoint too close to wall, forcing new one')
                        self.waypoint_generator.force_waypoint_change()
        
        except Exception as e:
            self.get_logger().error(f'Error in exploration loop: {str(e)}')

    def send_goal(self, goal_msg: PoseStamped):
        """Send navigation goal with proper error handling"""
        try:
            # Cancel any existing goal
            self.cancel_current_goal()
            
            # Create the goal
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal_msg
            
            # Add check for escape goal and clear emergency stop
            if self.is_escape_waypoint(goal_msg):
                self.get_logger().info('Escape goal detected - clearing emergency stop state')
                # Give the robot a moment to stabilize after emergency stop
                time.sleep(0.5)  # Short delay
                # Clear any velocity commands
                stop_cmd = Twist()
                self.wheel_speeds_pub.publish(stop_cmd)
            
            self.get_logger().info('Sending navigation goal:')
            self.get_logger().info(f'    Position: ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})')
            self.get_logger().info(f'    Frame: {goal_msg.header.frame_id}')
            self.get_logger().info(f'    Stamp: {goal_msg.header.stamp.sec}.{goal_msg.header.stamp.nanosec}')
            
            # Send the goal with timeout handling
            send_goal_future = self.nav_client.send_goal_async(
                nav_goal,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            # Store goal and update state
            self.current_goal = goal_msg
            self.is_navigating = True
            self.goal_start_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f'Error sending navigation goal: {str(e)}')
            self.reset_navigation_state()

    def goal_response_callback(self, future):
        """Handle the goal response with proper error handling"""
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                if self.is_escape_waypoint(self.current_goal):
                    self.handle_escape_failure("Goal rejected")
                else:
                    self.reset_navigation_state()
                return
            
            self.get_logger().info('Goal accepted')
            self._current_goal_handle = goal_handle
            
            # Get result future with timeout handling
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error in goal response: {str(e)}')
            self.reset_navigation_state()

    def get_result_callback(self, future):
        """Handle navigation result with timeout recovery"""
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f'Navigation result status: {status}')
            
            if result.result == NavigateToPose.Result.NO_VALID_PATH and self.is_escape_waypoint(self.current_goal):
                self.get_logger().warn('No valid path to escape!')
                self.start_spin_defense()

            elif status != GoalStatus.STATUS_SUCCEEDED and self.is_escape_waypoint(self.current_goal):
                self.handle_escape_failure("escape failed")
                self.escape_attempts += 1
                if self.escape_attempts < self.max_escape_attempts:
                    self.get_logger().warn(f'Retrying escape plan (attempt {self.escape_attempts + 1}/{self.max_escape_attempts})')
                    escape_point = self.human_avoidance.plan_escape()
                    if escape_point is not None:
                        self.send_goal(escape_point)  # Retry escape point
                    return
                else:
                    self.get_logger().error('Max escape attempts reached, giving up escape plan')
                    self.get_logger().warn('Escape plan failed')
                    self.start_escape_monitoring()
                    return
            else:                
                if status != GoalStatus.STATUS_SUCCEEDED:

                    self.get_logger().warn(f'Navigation failed with status: {status}')
                    # Normal failure handling
                    self.planning_attempts += 1
                    
                    if self.planning_attempts >= self.max_planning_attempts:
                        self.get_logger().warn('Max planning attempts reached, forcing new waypoint')
                        self.planning_attempts = 0
                        self.waypoint_generator.force_waypoint_change()
                else:
                    self.get_logger().info('Navigation succeeded')
                    if self.current_goal is not None and not self.is_escape_waypoint(self.current_goal):
                        self.planning_attempts = 0
                        self.reset_navigation_state()
                    if self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
                        self.get_logger().info('Escape plan succeeded - turning to face human')
                        self.cancel_current_goal()
                        self.reset_escape_state()
                        self.start_escape_monitoring()
                
                
        except Exception as e:
            self.get_logger().error(f'Error getting navigation result: {str(e)}')
            self.reset_navigation_state()

    def reset_escape_state(self):
        """Reset escape state"""
        self.escape_attempts = 0
        if self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
        
    def reset_navigation_state(self):
        """Reset navigation state and try new waypoint"""
        self.is_navigating = False
        self.current_goal = None
        self.goal_start_time = None
        # Force waypoint generator to pick new point if we've failed too many times
        if self.planning_attempts >= self.max_planning_attempts:
            self.planning_attempts = 0
            self.waypoint_generator.force_waypoint_change()
        # Force exploration loop to generate new waypoint
        self.exploration_loop()

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Log progress
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Navigation feedback - Distance remaining: '
            f'{feedback.distance_remaining:.2f}m'
        )

    def replan_to_waypoint(self):
        """Replan path to current waypoint after obstacle avoidance"""
        if self.current_goal:
            self.get_logger().info('Replanning path to waypoint after avoidance')
            self.send_goal(self.current_goal)
        else:
            self.exploration_loop()

    def cancel_current_goal(self):
        """Cancel the current navigation goal if one exists"""
        if self._current_goal_handle is not None:
            if self.is_escape_waypoint(self.current_goal):
                self.get_logger().info('Canceling escape goal')
                # Reset escape-specific state
                self.escape_attempts = 0
                self.is_tracking_human = False  # Ensure tracking stays off
            else:
                self.get_logger().info('Canceling exploration goal')
            
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self._current_goal_handle = None
            self.current_goal = None
            self.is_navigating = False

    def cancel_done_callback(self, future):
        """Handle goal cancellation result"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warn('Goal cancellation failed')

    def distance_to_goal(self, goal):
        """Calculate the distance to the goal"""
        return math.sqrt(
            (goal.pose.position.x - self.current_pose.pose.position.x) ** 2 +
            (goal.pose.position.y - self.current_pose.pose.position.y) ** 2
        )

    def check_goal_progress(self):
        """Monitor progress of current navigation goal"""
        if not self.is_navigating or self.current_goal is None:
            return
        
        try:
            #check the distance to the goal
            if not self.is_escape_waypoint(self.current_goal):
                if self.distance_to_goal(self.current_goal) < 0.5:
                    self.get_logger().info('Goal reached, cancelling...')
                    self.cancel_current_goal()
                    self.reset_navigation_state()
                    self.waypoint_generator.force_waypoint_change()
                    return
            elif self.is_escape_waypoint(self.current_goal):
                if self.distance_to_goal(self.current_goal) < 0.3:
                    self.get_logger().info('Escape goal reached, cancelling...')
                    self.cancel_current_goal()
                    self.reset_escape_state()
                    self.start_escape_monitoring()
                    return
            
            current_time = self.get_clock().now()
            time_navigating = (current_time - self.goal_start_time).nanoseconds / 1e9
            
            # Use longer timeout for escape waypoints
            timeout = self.escape_timeout if self.is_escape_waypoint(self.current_goal) else self.goal_timeout
            
            if time_navigating > timeout:
                self.get_logger().warn(f'Goal taking too long ({time_navigating:.1f}s), cancelling...')
                self.cancel_current_goal()
                
                if self.is_escape_waypoint(self.current_goal):
                    self.escape_attempts += 1
                    if self.escape_attempts < self.max_escape_attempts:
                        self.get_logger().warn(f'Retrying escape plan (attempt {self.escape_attempts + 1}/{self.max_escape_attempts})')
                        escape_point = self.human_avoidance.plan_escape()
                        if escape_point is not None:
                            self.send_goal(escape_point)  # Retry escape point
                            return
                        else:
                            self.get_logger().error('Failed to find escape point!')
                    else:
                        self.get_logger().error('Max escape attempts reached, giving up escape plan')
                        self.reset_escape_state()
                        self.cancel_current_goal()
                        self.start_escape_monitoring()
                # Normal waypoint handling
                else:
                    self.planning_attempts += 1
                    if self.planning_attempts >= self.max_planning_attempts:
                        self.get_logger().warn('Max planning attempts reached, forcing new waypoint')
                        self.planning_attempts = 0
                        self.waypoint_generator.force_waypoint_change()
                        self.reset_navigation_state()
                    else:
                        self.get_logger().info('Retrying current waypoint')
                        if self.current_goal:
                            self.send_goal(self.current_goal)
            
        except Exception as e:
            self.get_logger().error(f'Error checking goal progress: {str(e)}')

    def tracking_active_callback(self, msg):
        """Handle changes in tracking status"""
        was_tracking = self.is_tracking_human
        
        # Don't start tracking if we're executing an escape
        if self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
            self.get_logger().info('Ignoring tracking request - currently executing escape plan')
            self.is_tracking_human = False
            return
            
        self.is_tracking_human = msg.data
        
        if self.is_tracking_human and not was_tracking:
            # Cancel current navigation goal when starting to track
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()

    def tracking_cmd_callback(self, msg):
        """Handle human tracking commands with avoidance"""
        # First check if we're executing an escape plan
        if self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
            self.get_logger().info('Executing escape plan - ignoring ALL human tracking')
            return  # Don't process any human tracking while escaping
            
        if self.is_tracking_human:
            try:
                # Initialize avoidance_cmd before using it
                avoidance_cmd = Twist()
                
                # Extract human position from tracking command
                human_angle = -msg.angular.z  # Invert because cmd is opposite
                human_distance = msg.linear.y  # Get real distance measurement
                
                # Store human position whenever we see them
                if human_distance > 0:
                    # Convert polar to cartesian coordinates relative to robot
                    human_x = human_distance * math.cos(human_angle)
                    human_y = human_distance * math.sin(human_angle)
                    
                    try:
                        # Get robot's position in map frame
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'base_link',
                            rclpy.time.Time()
                        )
                        
                        # Transform human position to map coordinates
                        human_map_x = transform.transform.translation.x + human_x
                        human_map_y = transform.transform.translation.y + human_y
                        
                        # Store position with timestamp
                        self.last_human_position = (human_map_x, human_map_y)
                        self.last_human_timestamp = self.get_clock().now()
                        
                        self.get_logger().info(
                            f'Updated human position: ({human_map_x:.2f}, {human_map_y:.2f})'
                        )
                    except Exception as e:
                        self.get_logger().warn(f'Could not transform human position: {e}')
                
                # Calculate normalized image position from angular command
                image_x = ((-human_angle / self.max_angular_speed) + 1) * 320
                
                if human_distance > 0:  # Only if we have valid distance
                    # Get avoidance command
                    avoidance_cmd, needs_escape = self.human_avoidance.get_avoidance_command(
                        human_distance, 
                        human_angle,
                        image_x
                    )
                    
                    # IMPORTANT: Always publish the avoidance command
                    self.wheel_speeds_pub.publish(avoidance_cmd)
                    
                    # Check for escape BEFORE any other processing
                    if needs_escape:
                        self.get_logger().warn('Critical distance detected - initiating escape!')
                        
                        # Proceed with escape plan
                        self.cancel_current_goal()
                        if self.exploration_loop_timer:
                            self.exploration_loop_timer.cancel()
                        self.waypoint_generator.cancel_waypoint()  # Clear any exploration waypoints
                        self.update_human_position_in_map()
                        escape_point = self.human_avoidance.plan_escape()
                        
                        if escape_point is not None:
                            self.get_logger().info(
                                f'Got escape point at ({escape_point.pose.position.x:.2f}, '
                                f'{escape_point.pose.position.y:.2f})'
                            )
                            # Force tracking off BEFORE sending escape goal
                            self.is_tracking_human = False
                            self.send_goal(escape_point)
                            
                            return
                        else:
                            self.get_logger().error('Failed to get escape point!')
                    
                self.get_logger().info(
                    f'Human tracking: dist={human_distance:.2f}m, '
                    f'backing_up={avoidance_cmd.linear.x:.2f}m/s'
                )
                
            except Exception as e:
                self.get_logger().error(f'Error in tracking cmd callback: {str(e)}')
                self.wheel_speeds_pub.publish(Twist())  # Stop on error


    def is_escape_waypoint(self, waypoint):
        """Check if waypoint is an escape waypoint"""
        return waypoint is not None and waypoint.header.stamp.nanosec == 1

    def start_escape_monitoring(self):
        """Start monitoring after reaching escape point"""
        self.get_logger().info('Starting escape monitoring sequence')
        if self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
        self.escape_monitor_timer = self.create_timer(0.1, self.monitor_escape_sequence)

    def monitor_escape_sequence(self):
        """Monitor the escape sequence: turn -> resume"""
        try:
            if self.is_tracking_human:
                self.get_logger().info('Human detected, stopping turn.')
                self.wheel_speeds_pub.publish(Twist())  # Stop turning
                self.escape_again()  # Call escape again
                return
            
            # Calculate angle to last known human position
            elif self.last_human_position is not None:
                dx = self.last_human_position[0] - self.current_pose.pose.position.x
                dy = self.last_human_position[1] - self.current_pose.pose.position.y
                target_angle = math.atan2(dy, dx)
                if self.is_tracking_human:  # Assuming this variable indicates human detection
                    self.get_logger().info('Human detected, stopping turn.')
                    self.wheel_speeds_pub.publish(Twist())  # Stop turning
                    self.escape_again()  # Call escape again
                    return  # Exit the function to avoid further processing
                
                # Get rotation speeds from human avoidance controller
                cmd = self.human_avoidance.turn_to_angle(target_angle)
                self.wheel_speeds_pub.publish(cmd)
                
                # Check if we have reached the target angle
                if abs(cmd.angular.z) < 0.01:
                    self.get_logger().info('Turned to face last known human position, resuming exploration')
                    self.cleanup_escape_monitoring()
                    self.resume_exploration()
                    return
            else:
                # No known human position, cleanup and resume
                self.cleanup_escape_monitoring()
                self.resume_exploration()
                return
            
        except Exception as e:
            self.get_logger().error(f'Error in escape monitoring: {str(e)}')
            self.cleanup_escape_monitoring()
            self.resume_exploration()

    def cleanup_escape_monitoring(self):
        """Clean up escape monitoring timers and state"""
        if self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
            self.escape_monitor_timer = None
        self.get_logger().info('Cleaned up escape monitoring')

    def resume_exploration(self):
        """Clean up escape monitoring and resume exploration"""
        self.cleanup_escape_monitoring()  # Make sure monitoring is cleaned up
        self.exploration_loop_timer.reset()
        self.waypoint_generator.force_waypoint_change()
        self.reset_navigation_state()

    def escape_again(self):
        """Escape again"""
        self.get_logger().info('Escape again')
        if self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
            self.reset_escape_state()
        
    def update_human_position_in_map(self):
        """Update the occupancy grid with the last known human position"""
        if self.current_map is None or self.last_human_position is None:
            return

        try:
            # Convert human position to map coordinates
            map_x = int((self.last_human_position[0] - self.current_map.info.origin.position.x) / 
                        self.current_map.info.resolution)
            map_y = int((self.last_human_position[1] - self.current_map.info.origin.position.y) / 
                        self.current_map.info.resolution)

            # Define radius of human obstacle (in meters)
            human_radius = 0.23  # 0.25 meter radius
            cells_radius = int(human_radius / self.current_map.info.resolution)

            # Create temporary map data
            temp_map = list(self.current_map.data)

            # Mark cells around human position as occupied
            for dx in range(-cells_radius, cells_radius + 1):
                for dy in range(-cells_radius, cells_radius + 1):
                    # Check if point is within circular radius
                    if dx*dx + dy*dy <= cells_radius*cells_radius:
                        cell_x = map_x + dx
                        cell_y = map_y + dy
                        
                        # Check if coordinates are within map bounds
                        if (0 <= cell_x < self.current_map.info.width and 
                            0 <= cell_y < self.current_map.info.height):
                            # Calculate index in flattened array
                            index = cell_y * self.current_map.info.width + cell_x
                            # Mark as occupied (100 represents occupied in occupancy grid)
                            temp_map[index] = 100

            # Update map data
            self.current_map.data = temp_map
            
            # Republish modified map
            self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
            self.map_pub.publish(self.current_map)
            
            self.get_logger().info(
                f'Updated map with human obstacle at ({self.last_human_position[0]:.2f}, '
                f'{self.last_human_position[1]:.2f})'
            )

        except Exception as e:
            self.get_logger().error(f'Error updating human position in map: {str(e)}')

    def start_spin_defense(self):
        """Start spinning in place as last resort defense"""
        self.get_logger().warn('Starting spin defense!')
        self.is_spinning = True
        
        # Cancel any existing navigation
        self.cancel_current_goal()
        if self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
        if self.spin_timer:
            self.spin_timer.cancel()
        self.spin_timer = self.create_timer(0.1, self.spin_defense_callback)

    def spin_defense_callback(self):
        """Execute spin movement"""
        if not self.is_spinning:
            if self.spin_timer:
                self.spin_timer.cancel()
                self.spin_timer = None
            return
        
        cmd = Twist()
        cmd.angular.z = self.SPIN_SPEED
        self.wheel_speeds_pub.publish(cmd)

    def handle_escape_failure(self, reason):
        """Handle failed escape attempt"""
        self.get_logger().warn(f'Escape failed: {reason}')
        

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
