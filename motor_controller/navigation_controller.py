#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
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
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.msg import Costmap
from std_msgs.msg import Header
import struct
from .processors.human_escape import HumanEscape

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
        self.los_debug_pub = self.create_publisher(MarkerArray, 'los_debug_markers', 10)
        
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
        self.goal_timeout = 20.0  # 20 seconds total timeout per goal
        self.planning_attempts = 0
        self.max_planning_attempts = 2  # Max attempts before giving up
        self.goal_start_time = None

        self.shake_timer = None
        
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
            PoseStamped,
            '/human_coords',
            self.tracking_cmd_callback,
            10
        )
        
        self.is_tracking_human = False
        
        self.get_logger().info('Navigation controller initialized')
        
        self.current_goal_handle = None

        self.human_avoidance = HumanAvoidanceController(self, self.waypoint_generator)

        # Add escape-specific parameters
        self.escape_timeout = 15.0  # Longer timeout for escape attempts
        self.max_escape_attempts = 2  # Number of retry attempts for escape
        self.escape_attempts = 0  # Counter for escape attempts
        
        # Add storage for last seen human position
        self.last_human_position = None
        self.last_human_timestamp = None
        self.turn_timeout = 10.0

        # Add timer for escape monitoring (initially disabled)
        self.escape_monitor_timer = None

        #Previous escape waypoint
        self.previous_escape_waypoint_failed = False

        # Add map publisher for human obstacle updates
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)

        # Create publisher for human obstacles with proper frame
        self.human_obstacles_pub = self.create_publisher(
            PointCloud2, 
            '/human_obstacles',
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        
        # Add timestamp tracking for human obstacle persistence
        self.human_obstacle_timeout = 2.0  # Keep obstacles for 2 seconds
        
        # Create timer to periodically update human obstacles
        # self.obstacle_update_timer = self.create_timer(0.2, self.update_human_obstacles)

        # Add a service client for triggering path replanning
        self.make_plan_client = self.create_client(Empty, '/global_costmap/global_costmap/clear_except_static')

        # Add position tracking for stuck detection
        self.last_position_check = None
        self.last_check_position = None
        self.stuck_threshold = 0.05  # 5cm movement threshold
        self.stuck_timeout = 10.0     # 5 seconds without movement = stuck

        # Add tracking timeout parameters
        self.human_tracking_timeout = 3.0  # Wait 2 seconds before ending tracking

        # Add parameter for explicitly clearing costmaps
        self.clear_costmaps_after_escape = True
        self.clear_after_human = True

        # Add after other initializations
        self.is_executing_escape = False

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
        # First check if we've lost track of human
        if self.check_tracking_timeout():
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
                    self.reset_escape_state()
                else:
                    self.reset_navigation_state()
                return
            
            self.get_logger().info('Goal accepted')
            self.current_goal_handle = goal_handle
            
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

            # Check if human is still present
            human_still_present = False
            current_time = self.get_clock().now()
            if self.last_human_timestamp is not None:
                time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
                # Consider human still present if seen in the last 2 seconds
                human_still_present = time_since_human < 2.0
            
            if status != GoalStatus.STATUS_SUCCEEDED and self.is_escape_waypoint(self.current_goal):
                self.escape_attempts += 1
                if self.escape_attempts < self.max_escape_attempts:
                    self.get_logger().warn(f'Retrying escape plan (attempt {self.escape_attempts + 1}/{self.max_escape_attempts})')
                    escape_point = self.human_avoidance.plan_escape(self.previous_escape_waypoint_failed)
                    time.sleep(0.5)
                    if escape_point is not None:
                        self.send_goal(escape_point)
                    else:
                        self.get_logger().error('Failed to find escape point!')
                        self.previous_escape_waypoint_failed = True
                elif self.escape_attempts >= self.max_escape_attempts and human_still_present:
                    self.get_logger().info('Trapped start shaking')
                    self.cancel_current_goal()
                    time.sleep(0.5)
                    self.start_shake_defense()
                    return
                else:
                    self.get_logger().error('Max escape attempts reached, giving up escape plan')
                    self.get_logger().warn('Escape plan failed')
                    self.cancel_current_goal()
                    return
            elif status != GoalStatus.STATUS_SUCCEEDED and not self.is_escape_waypoint(self.current_goal):
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
                elif self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
                    self.get_logger().info('Escape plan succeeded - turning to face human')
                    self.reset_escape_state()
                    self.start_escape_monitoring()
                
                
        except Exception as e:
            self.get_logger().error(f'Error getting navigation result: {str(e)}')
            self.reset_navigation_state()

    def reset_goal_state(self):
        """Reset all goal-related state"""
        self.current_goal = None
        self.is_navigating = False
        self.goal_start_time = None
        self.current_goal_handle = None
        # Reset stuck detection
        self.last_position_check = None
        self.last_check_position = None

    def reset_escape_state(self):
        """Reset all escape-related state"""
        self.reset_goal_state()  # Reset base goal state first
        self.escape_attempts = 0
        self.previous_escape_waypoint_failed = False
        
        # If there's an escape monitor running, cancel it
        if hasattr(self, 'escape_monitor_timer') and self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
            self.escape_monitor_timer = None
        
        self.get_logger().info('Escape state reset')

    def reset_navigation_state(self):
        """Reset navigation state and try new waypoint"""
        self.reset_goal_state()  # Reset base goal state first
        
        # Force waypoint generator to pick new point if we've failed too many times
        if self.planning_attempts >= self.max_planning_attempts:
            self.planning_attempts = 0
            self.waypoint_generator.force_waypoint_change()
        else:
            # Reset previous waypoint to avoid comparison issues
            self.previous_waypoint = None
            
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

    def cancel_current_goal(self):
        """Cancel the current navigation goal if one exists"""
        if self.current_goal_handle is not None:
            self.clear_visualization_markers()
            if self.is_escape_waypoint(self.current_goal):
                self.get_logger().info('Canceling escape goal')
                # Reset escape-specific state
                self.is_tracking_human = False  # Ensure tracking stays off
            else:
                self.get_logger().info('Canceling exploration goal')
            
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.current_goal_handle = None
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
            # Check if human is still present
            human_still_present = False
            current_time = self.get_clock().now()
            if self.last_human_timestamp is not None:
                time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
                # Consider human still present if seen in the last 2 seconds
                human_still_present = time_since_human < 2.0

            # Check for overall goal timeout
            goal_timeout_reached = False
            if self.goal_start_time is not None:
                goal_duration = (current_time - self.goal_start_time).nanoseconds / 1e9
                goal_timeout_reached = goal_duration > self.goal_timeout
                
                if goal_timeout_reached:
                    self.get_logger().warn(
                        f'Goal timeout exceeded! {goal_duration:.1f}s elapsed (limit: {self.goal_timeout}s)'
                    )

            """Check if the robot has moved in the past interval"""
            if self.current_goal is None:
                # Reset tracking when not navigating
                self.last_position_check = None
                self.last_check_position = None
                return
            
            current_position = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
            
            # Initialize tracking on first call
            if self.last_position_check is None or self.last_check_position is None:
                self.last_position_check = current_time
                self.last_check_position = current_position
                return
            
            # Calculate time and distance since last check
            time_diff = (current_time - self.last_position_check).nanoseconds / 1e9
            distance_moved = math.sqrt(
                (current_position[0] - self.last_check_position[0]) ** 2 +
                (current_position[1] - self.last_check_position[1]) ** 2
            )
            
            # Check if we've been stuck for longer than the timeout OR if goal timeout was reached
            if (distance_moved < self.stuck_threshold and time_diff > self.stuck_timeout) or goal_timeout_reached:
                if not goal_timeout_reached:
                    self.get_logger().warn(
                        f'Robot appears to be stuck! Moved only {distance_moved:.3f}m in {time_diff:.1f} seconds'
                    )
                
                # Different handling based on goal type
                if self.is_escape_waypoint(self.current_goal):
                    self.escape_attempts += 1
                    if self.escape_attempts < self.max_escape_attempts:
                        self.get_logger().warn(f'Retrying escape plan (attempt {self.escape_attempts + 1}/{self.max_escape_attempts})')
                        
                        # Pass the failure flag to plan_escape
                        escape_point = self.human_avoidance.plan_escape(self.previous_escape_waypoint_failed)
                        
                        if escape_point is not None:
                            self.send_goal(escape_point)  # Retry escape point
                        else:
                            self.get_logger().error('Failed to find escape point!')
                    elif self.escape_attempts >= self.max_escape_attempts and human_still_present:
                        self.get_logger().info('Trapped')
                        self.cancel_current_goal()
                        time.sleep(0.5)
                        self.start_shake_defense()
                    else:
                        self.get_logger().error('Max escape attempts reached, giving up escape plan')
                        self.cancel_current_goal(failed_escape=True)
                        self.reset_escape_state()
                else:
                    self.planning_attempts += 1
                    if self.planning_attempts >= self.max_planning_attempts:
                        self.get_logger().warn('Max planning attempts reached, forcing new waypoint')
                        self.planning_attempts = 0
                        self.cancel_current_goal()
                        self.waypoint_generator.force_waypoint_change()
                        self.reset_navigation_state()
                    else:
                        self.get_logger().info('Retrying current waypoint')
                        if self.current_goal:
                            self.send_goal(self.current_goal)
                
                # Reset tracking
                self.last_position_check = None
                self.last_check_position = None
                self.get_logger().info('Reset tracking')
            
            # Update tracking if we've moved enough or enough time has passed
            elif distance_moved > self.stuck_threshold or time_diff > 10.0:
                self.last_position_check = current_time
                self.last_check_position = current_position
            
            # Add dynamic escape path monitoring
            if self.is_escape_waypoint(self.current_goal) and hasattr(self.human_avoidance, 'waypoint_generator'):
                # Check if we have a HumanEscape generator
                waypoint_generator = self.human_avoidance.waypoint_generator
                if isinstance(waypoint_generator, HumanEscape):
                    # Check if human is intercepting and we need a new escape path
                    new_escape_point = waypoint_generator.check_and_update_escape_if_needed()
                    if new_escape_point is not None:
                        self.get_logger().warn('Updating escape path - human interception detected')
                        self.send_goal(new_escape_point)
                        return
            
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
            self.cancel_current_goal()

    def tracking_cmd_callback(self, msg: PoseStamped):
        """Handle tracking information from human coordinates"""
        try:
            # Extract human position from PoseStamped
            human_x = msg.pose.position.x
            human_y = msg.pose.position.y
            
            # Always update last known human position and timestamp, even during escape
            self.last_human_position = (human_x, human_y)
            self.last_human_timestamp = self.get_clock().now()

            if self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
                return
            if self.shake_timer:
                return
            
            # Calculate distance to human using Euclidean distance
            if self.current_pose is not None:
                dx = human_x - self.current_pose.pose.position.x
                dy = human_y - self.current_pose.pose.position.y
                human_distance = math.sqrt(dx*dx + dy*dy)
                
                # Calculate angle to human
                human_angle = math.atan2(dy, dx)
                
                # Log human information
                self.get_logger().info(
                    f'Human detected at ({human_x:.2f}, {human_y:.2f}), '
                    f'distance: {human_distance:.2f}m, '
                    f'angle: {math.degrees(human_angle):.1f}°'
                )
                
                # Pass information to human avoidance controller
                if self.is_tracking_human:
                    cmd_vel = Twist()
                    
                    # UPDATED: Use direct pose and position instead of image_x
                    cmd_vel, should_escape = self.human_avoidance.get_avoidance_command(
                        human_distance, 
                        human_angle,
                        robot_pose=self.current_pose,
                        human_pos=self.last_human_position
                    )
                    
                    # Always publish the avoidance command
                    self.wheel_speeds_pub.publish(cmd_vel)
                    
                    # Log command details
                    self.get_logger().info(
                        f'Human tracking: dist={human_distance:.2f}m, '
                        f'angle={human_angle:.2f}rad, turn={cmd_vel.angular.z:.3f}'
                    )
                    
                    # Check for escape BEFORE any other processing                    
                    if should_escape:
                        self.get_logger().warn('Critical distance detected - initiating escape!')
                        
                        # Cancel current navigation goal and exploration
                        self.cancel_current_goal()
                        if self.exploration_loop_timer:
                            self.exploration_loop_timer.cancel()
                        self.waypoint_generator.cancel_waypoint()  # Clear any exploration waypoints
                    
                        escape_point = self.human_avoidance.plan_escape()
                        
                        if escape_point is not None:
                            self.get_logger().info(
                                f'Got escape point at ({escape_point.pose.position.x:.2f}, '
                                f'{escape_point.pose.position.y:.2f})'
                            )
                            # Force tracking off BEFORE sending escape goal
                            self.is_tracking_human = False
                            self.send_goal(escape_point)
                            
                            # Reset escape attempts counter for fresh start
                            self.escape_attempts = 0

                            return
                        else:
                            self.get_logger().error('Failed to get escape point!')
                    
                    self.get_logger().info(
                        f'Human tracking: dist={human_distance:.2f}m, '
                        f'backing_up={cmd_vel.linear.x:.2f}m/s'
                    )

        except Exception as e:
            self.get_logger().error(f'Error in tracking command callback: {str(e)}')
            self.wheel_speeds_pub.publish(Twist())  # Stop on error

    def is_escape_waypoint(self, waypoint):
        """Check if waypoint is an escape waypoint"""
        return waypoint is not None and waypoint.header.stamp.nanosec == 1

    def start_escape_monitoring(self):
        """Start monitoring after reaching escape point"""
        self.get_logger().info('Starting escape monitoring sequence')
        if self.exploration_loop_timer:
            self.exploration_loop_timer.cancel()
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
                
                # Get rotation speeds from human avoidance controller
                cmd = self.human_avoidance.turn_to_angle(target_angle)  # Fix: only get cmd, not turn_time
                self.wheel_speeds_pub.publish(cmd)
                
                # Check if we've been trying to turn for too long or if we're done turning
                current_time = self.get_clock().now()
                
                # Initialize turn start time if not set
                if not hasattr(self, 'turn_start_time') or self.turn_start_time is None:
                    self.turn_start_time = current_time
                    
                # Calculate elapsed time
                turn_time = (current_time - self.turn_start_time).nanoseconds / 1e9
                
                # Check if we have reached the target angle or timed out
                if abs(cmd.angular.z) < 0.01 or turn_time > self.turn_timeout:
                    self.get_logger().info('Turned to face last known human position, resuming exploration')
                    # Reset turn timer
                    self.turn_start_time = None
                    time.sleep(2)
                    if self.is_tracking_human:
                        return
                    else:
                        self.cleanup_escape_monitoring()
                        self.resume_exploration()
                    return
            else:
                # No known human position, cleanup and resume
                self.get_logger().info("No last know position")
                self.turn_start_time = None  # Reset turn timer
                self.cleanup_escape_monitoring()
                self.resume_exploration()
                return
            
        except Exception as e:
            self.get_logger().error(f'Error in escape monitoring: {str(e)}')
            self.turn_start_time = None  # Reset turn timer
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
        self.escape_attempts = 0  # Reset escape attempts
        
        # Clear old markers before resuming
        self.clear_visualization_markers()
        
        self.exploration_loop_timer.reset()
        self.reset_navigation_state()

    def escape_again(self):
        """Escape again"""
        self.get_logger().info('Escape again')
        if self.escape_monitor_timer:
            self.escape_monitor_timer.cancel()
            self.reset_escape_state()

    def clear_visualization_markers(self):
        """Clear all visualization markers"""
        try:
            # Create an empty marker array
            marker_array = MarkerArray()
            
            # Add a deletion marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            
            # Publish the deletion marker
            self.marker_pub.publish(marker_array)
            self.get_logger().debug('Cleared visualization markers')
        except Exception as e:
            self.get_logger().error(f'Error clearing markers: {str(e)}')
            
    def publish_human_obstacle(self, radius=0.25):
        """Publish human obstacle positions as PointCloud2 with specified radius"""
        if self.last_human_position is None:
            return
        
        # Check if we should still show the obstacle
        if self.last_human_timestamp is not None:
            current_time = self.get_clock().now()
            time_since_detection = (current_time - self.last_human_timestamp).nanoseconds / 1e9
            
            if time_since_detection > self.human_obstacle_timeout:
                # Clear the obstacle after timeout
                self.get_logger().info('Clearing human obstacle - detection timeout')
                return
        
        try:
            # Create point cloud message
            pc2 = PointCloud2()
            pc2.header.stamp = self.get_clock().now().to_msg()
            pc2.header.frame_id = "map"
            
            # Define fields for x, y, z coordinates
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            pc2.fields = fields
            
            # Generate dense point cloud around human
            points = []
            human_x, human_y = self.last_human_position
            
            # Create a solid circle obstacle instead of a donut
            resolution = 0.03  # Higher resolution for smoother obstacle
            
            # Use multiple height levels for the voxel representation (between 0.1m and 1.7m)
            height_levels = [0.1, 0.4, 0.7, 1.0, 1.3, 1.7]  # Represent human at various heights
            
            for height in height_levels:
                for dx in np.arange(-radius, radius + resolution, resolution):
                    for dy in np.arange(-radius, radius + resolution, resolution):
                        dist_sq = dx*dx + dy*dy
                        # Only add points within radius (solid circle)
                        if dist_sq <= radius*radius:
                            # Calculate intensity - higher at center, slightly lower at edges
                            dist = math.sqrt(dist_sq)
                            intensity_ratio = dist / radius  # 0 at center, 1 at edge
                            # Use 254 (lethal) near center, decreasing to 245 at edges
                            intensity = 254.0 - (1.0 * intensity_ratio)
                            
                            # Adjust for fade based on time
                            if self.last_human_timestamp is not None:
                                time_since_detection = (self.get_clock().now() - self.last_human_timestamp).nanoseconds / 1e9
                                fade_ratio = max(0.0, (self.human_obstacle_timeout - time_since_detection) / self.human_obstacle_timeout)
                                intensity *= fade_ratio
                            
                            # Add point with proper z-coordinate 
                            points.append((human_x + dx, human_y + dy, height, intensity))

            # Pack points into PointCloud2
            pc2.height = 1
            pc2.width = len(points)
            pc2.point_step = 16  # 4 fields * 4 bytes
            pc2.row_step = pc2.point_step * pc2.width
            pc2.is_dense = True
            
            # Pack points into binary array
            point_data = bytearray()
            for p in points:
                point_data.extend(struct.pack('ffff', *p))
            pc2.data = point_data
            
            # Publish point cloud
            self.human_obstacles_pub.publish(pc2)

            # Add time information to logging
            time_info = ""
            if self.last_human_timestamp is not None:
                time_since = (self.get_clock().now() - self.last_human_timestamp).nanoseconds / 1e9
                time_info = f" (last seen {time_since:.1f}s ago)"
            
            self.get_logger().info(
                f'Published solid circle human obstacle at ({human_x:.2f}, {human_y:.2f}) '
                f'with radius {radius}m{time_info}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing human obstacle: {str(e)}')

    def update_human_obstacles(self):
        """Periodically update human obstacle representation"""
        if self.last_human_position is not None and self.last_human_timestamp is not None:
            self.publish_human_obstacle()

    def start_shake_defense(self):
        """Start a shaking motion to try to escape when trapped"""
        
        # Don't start shake defense if an escape is in progress
        if self.is_executing_escape:
            self.get_logger().info('Escape plan already in progress, not starting shake defense')
            return
        
        self.get_logger().warn('Starting shake defense - robot is trapped!')
        
        # Cancel any current navigation goals
        self.cancel_current_goal()
        
        # Create a timer for the shake motion
        self.shake_count = 0
        self.shake_direction = 1  # Start with right turn
        
        # Create a timer that runs the shake motion at 5Hz
        if hasattr(self, 'shake_timer') and self.shake_timer:
            self.shake_timer.cancel()
        self.shake_timer = self.create_timer(0.2, self.execute_shake_motion)
        
        self.get_logger().info('Shake defense initiated')

    def request_costmap_clear(self):
        """Request clearing of the global costmap except static layer"""
        try:
            # Create service client for clearing costmap
            clear_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            
            # Wait for service to be available with short timeout
            if not clear_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn('Clear costmap service not available, skipping clear')
                return False
                
            # Create request
            request = ClearEntireCostmap.Request()
            
            # Call service asynchronously
            future = clear_client.call_async(request)
            
            # Log the request
            self.get_logger().info('Requested costmap clear for escape planning')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to clear costmap: {str(e)}')
            return False
        
    def execute_shake_motion(self):
        """Execute one step of the shake motion"""
        try:
            # Check if we're currently trying to escape - if so, don't shake
            if self.is_executing_escape:
                self.get_logger().info('Escape plan in progress, not executing shake motion')
                if hasattr(self, 'shake_timer') and self.shake_timer:
                    self.shake_timer.cancel()
                    self.shake_timer = None
                return
            
            # Check if human is still present
            current_time = self.get_clock().now()
            human_still_present = False
            
            if self.last_human_timestamp is not None:
                time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
                # Consider human still present if seen in the last 2 seconds
                human_still_present = time_since_human < 3.0
            
            if not human_still_present:
                # Human is gone, we can stop shaking
                self.get_logger().info('Human no longer detected, stopping shake defense')
                self.wheel_speeds_pub.publish(Twist())  # Stop motion
                
                # Clear costmaps after human is gone
                self.request_costmap_clear()
                self.get_logger().info('Clearing costmaps after human departed')
                
                if hasattr(self, 'shake_timer') and self.shake_timer:
                    self.shake_timer.cancel()
                    self.shake_timer = None
                
                # Reset escape state and resume exploration
                self.reset_escape_state()
                self.resume_exploration()
                return
            
            # Create shake command
            cmd = Twist()
            
            # Alternate between turning left and right with some forward/backward motion
            if self.shake_count % 2 == 0:
                # Even counts: turn with some linear motion
                cmd.angular.z = 0.8 * self.shake_direction
            else:
                # Odd counts: turn the other way
                self.shake_direction *= -1  # Flip direction
                cmd.angular.z = 0.8 * self.shake_direction
            
            # Publish command
            self.wheel_speeds_pub.publish(cmd)
            self.get_logger().info(f'Shake motion' + f'angular={cmd.angular.z:.2f}, linear={cmd.linear.x:.2f}')
            
            # Increment counter
            self.shake_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error in shake motion: {str(e)}')
            # Stop motion on error
            self.wheel_speeds_pub.publish(Twist())
            if hasattr(self, 'shake_timer') and self.shake_timer:
                self.shake_timer.cancel()
                self.shake_timer = None
            self.reset_escape_state()

    def check_tracking_timeout(self):
        """Check if we should stop tracking due to not seeing human"""
        if not self.is_tracking_human or self.last_human_timestamp is None:
            return False
        
        current_time = self.get_clock().now()
        time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
        
        if time_since_human < self.human_tracking_timeout:
            return True
        else:
            return False


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
