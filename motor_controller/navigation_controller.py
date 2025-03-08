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
from .processors.navigation_fsm import NavigationFSM, NavigationState, NavigationEvent

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
        
        # State variables
        self.latest_scan = None
        self.current_goal = None
        self.is_navigating = False
        self.goal_start_time = None
        self.previous_waypoint = None

        # Add state for Nav2 readiness
        self.nav2_ready = False
        
        # Add timeout parameters
        self.goal_timeout = 20.0  # 20 seconds total timeout per goal
        self.planning_attempts = 0
        self.max_planning_attempts = 2  # Max attempts before giving up
        self.goal_start_time = None

        self.shake_timer = None
        
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

        # Add escapo monitor timer (initially disabled)
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

        # Add these parameters after other initializations
        self.tf_timeout = 0.1  # Short timeout for transform lookups
        self.tf_retry_count = 3  # Number of retries for transform lookups
        self.tf_fallback_to_latest = True  # Use latest available transform if requested time is not available
        self.tf_use_sim_time = False  # Whether using simulation time
        self.tf_last_error_time = self.get_clock().now()  # Track last error time to avoid spamming logs
        
        # Set up FSM with callbacks
        self.fsm_callbacks = {
            "on_enter_initializing": self.on_enter_initializing,
            "on_enter_idle": self.on_enter_idle,
            "on_enter_exploring": self.on_enter_exploring,
            "on_enter_human_tracking": self.on_enter_human_tracking,
            "on_enter_escaping": self.on_enter_escaping,
            "on_enter_shake_defense": self.on_enter_shake_defense,
            "on_enter_post_escape": self.on_enter_post_escape,
            "on_enter_error": self.on_enter_error,
            
            "on_exit_initializing": self.on_exit_initializing,
            "on_exit_idle": self.on_exit_idle,
            "on_exit_exploring": self.on_exit_exploring,
            "on_exit_human_tracking": self.on_exit_human_tracking,
            "on_exit_escaping": self.on_exit_escaping,
            "on_exit_shake_defense": self.on_exit_shake_defense,
            "on_exit_post_escape": self.on_exit_post_escape,
            "on_exit_error": self.on_exit_error,
            
            "on_update_any": self.on_update_any
        }
        
        # Create the FSM
        self.fsm = NavigationFSM(self, self.fsm_callbacks)

    # FSM Callbacks

    def on_enter_initializing(self, event=None, data=None):
        self.get_logger().info("Initializing navigation system")
        
    def on_exit_initializing(self, event=None, data=None):
        self.get_logger().info("Initialization complete")
        
    def on_enter_idle(self, event=None, data=None):
        self.get_logger().info("Entering idle state")
        # Reset navigation state
        self.reset_goal_state()
        
    def on_exit_idle(self, event=None, data=None):
        self.get_logger().info("Exiting idle state")
        
    def on_enter_exploring(self, event=None, data=None):
        self.get_logger().info("Starting exploration")
        # Start generating waypoints
        waypoint = self.waypoint_generator.generate_waypoint()
        if waypoint:
            self.send_goal(waypoint)
            
    def on_exit_exploring(self, event=None, data=None):
        self.get_logger().info("Stopping exploration")
        self.cancel_current_goal()
        
    def on_enter_human_tracking(self, event=None, data=None):
        self.get_logger().info("Starting human tracking")
        self.is_tracking_human = True
        self.cancel_current_goal()
        
    def on_exit_human_tracking(self, event=None, data=None):
        self.get_logger().info("Stopping human tracking")
        self.is_tracking_human = False
        
    def on_enter_escaping(self, event=None, data=None):
        self.get_logger().info("Starting escape sequence")
        escape_point = self.human_avoidance.plan_escape()
        if escape_point is not None:
            self.send_goal(escape_point)
            self.escape_attempts = 0
        else:
            self.get_logger().error("Failed to plan escape route")
            self.fsm.trigger_event(NavigationEvent.TRAPPED)
            
    def on_exit_escaping(self, event=None, data=None):
        self.get_logger().info("Exiting escape sequence")
        
    def on_enter_shake_defense(self, event=None, data=None):
        self.get_logger().info("Starting shake defense")
        self.cancel_current_goal()
        self.shake_count = 0
        self.shake_direction = 1
        
    def on_exit_shake_defense(self, event=None, data=None):
        self.get_logger().info("Stopping shake defense")
        # Stop any motion
        self.wheel_speeds_pub.publish(Twist())
        
    def on_enter_post_escape(self, event=None, data=None):
        self.get_logger().info("Post-escape: turning to face human")
        # Logic for turning to face the last known human position
        
    def on_exit_post_escape(self, event=None, data=None):
        self.get_logger().info("Exiting post-escape state")
        
    def on_enter_error(self, event=None, data=None):
        self.get_logger().error(f"Error state: {data if data else 'Unknown error'}")
        # Try to recover to a safe state
        self.cancel_current_goal()
        self.wheel_speeds_pub.publish(Twist())  # Stop motion
        
    def on_exit_error(self, event=None, data=None):
        self.get_logger().info("Recovered from error state")
        
    def on_update_any(self, state=None, event=None, data=None):
        # Common update logic for all states
        pass

    # Core functionality methods
    def get_current_pose(self):
        """Get current robot pose with robust transform handling"""
        try:
            # Try multiple times with increasing timeouts
            for attempt in range(self.tf_retry_count):
                try:
                    # Use current time for transform lookup
                    current_time = self.get_clock().now()
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time(),  # Use latest available transform
                        timeout=rclpy.duration.Duration(seconds=self.tf_timeout * (attempt + 1))
                    )
                    
                    # Create pose from transform
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = current_time.to_msg()
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    
                    # Update current pose
                    self.current_pose = pose
                    return pose
                    
                except TransformException:
                    # Log warning only on last attempt to avoid spamming
                    if attempt == self.tf_retry_count - 1:
                        # Rate limit error messages
                        current_time = self.get_clock().now()
                        if (current_time - self.tf_last_error_time).nanoseconds / 1e9 > 5.0:  # Only log every 5 seconds
                            self.get_logger().warn(f'Transform lookup failed after {attempt+1} attempts. Using last known pose.')
                            self.tf_last_error_time = current_time
                    continue
            
            # If all attempts failed, return last known pose
            return self.current_pose
            
        except Exception as e:
            # Rate limit error messages
            current_time = self.get_clock().now()
            if (current_time - self.tf_last_error_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().error(f'Error getting current pose: {str(e)}')
                self.tf_last_error_time = current_time
            return self.current_pose

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data with robust transform handling"""
        self.latest_scan = msg
        
        # Update current_pose using the improved method
        self.current_pose = self.get_current_pose()
        
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
                self.get_logger().warn(f"Escape navigation failed with status {status}")
                
                if self.fsm.is_in_state(NavigationState.ESCAPING):
                    self.fsm.trigger_event(NavigationEvent.ESCAPE_FAILED)
                
            elif status != GoalStatus.STATUS_SUCCEEDED and not self.is_escape_waypoint(self.current_goal):
                self.get_logger().warn(f'Navigation failed with status: {status}')
                
                if self.fsm.is_in_state(NavigationState.EXPLORING):
                    self.fsm.trigger_event(NavigationEvent.GOAL_FAILED)
                
            else:
                self.get_logger().info('Navigation succeeded')
                
                if self.is_escape_waypoint(self.current_goal):
                    if self.fsm.is_in_state(NavigationState.ESCAPING):
                        self.fsm.trigger_event(NavigationEvent.ESCAPE_SUCCEEDED)
                else:
                    if self.fsm.is_in_state(NavigationState.EXPLORING):
                        self.fsm.trigger_event(NavigationEvent.GOAL_REACHED)
                
        except Exception as e:
            self.get_logger().error(f'Error getting navigation result: {str(e)}')
            self.fsm.trigger_event(NavigationEvent.ERROR_OCCURRED, str(e))

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
            
        self.get_logger().info('Navigation state reset')

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
        try:
            if self.current_goal_handle is not None:
                self.clear_visualization_markers()
                
                # Log what type of goal we're cancelling
                if self.is_escape_waypoint(self.current_goal):
                    self.get_logger().info('Canceling escape goal')
                else:
                    self.get_logger().info('Canceling exploration goal')
                
                # Send cancel request without callback
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().error(f'Error sending cancel request: {str(e)}')
                
                # Reset goal tracking state
                self.current_goal_handle = None
                self.current_goal = None
                self.is_navigating = False
                
                return True
            else:
                self.get_logger().info('No active goal to cancel')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Error cancelling goal: {str(e)}')
            # Still reset state on error
            self.current_goal_handle = None
            self.current_goal = None
            self.is_navigating = False
            return False

    def distance_to_goal(self, goal):
        """Calculate the distance to the goal"""
        return math.sqrt(
            (goal.pose.position.x - self.current_pose.pose.position.x) ** 2 +
            (goal.pose.position.y - self.current_pose.pose.position.y) ** 2
        )

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
            # Update FSM state when tracking starts
            self.fsm.trigger_event(NavigationEvent.HUMAN_DETECTED)
        elif not self.is_tracking_human and was_tracking:
            # Update FSM state when tracking stops
            self.fsm.trigger_event(NavigationEvent.HUMAN_LOST)

    def tracking_cmd_callback(self, msg: PoseStamped):
        """Handle tracking information from human coordinates"""
        try:
            # Extract human position from PoseStamped
            human_x = msg.pose.position.x
            human_y = msg.pose.position.y
            
            # Always update last known human position and timestamp, even during escape
            self.last_human_position = (human_x, human_y)
            self.last_human_timestamp = self.get_clock().now()

            if self.fsm.is_in_state(NavigationState.ESCAPING) or self.fsm.is_in_state(NavigationState.SHAKE_DEFENSE):
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
                
                # Pass information to human avoidance controller if we're in tracking state
                if self.fsm.is_in_state(NavigationState.HUMAN_TRACKING):
                    cmd_vel = Twist()
                    
                    # Use direct pose and position
                    cmd_vel, should_escape = self.human_avoidance.get_avoidance_command(
                        human_distance, 
                        human_angle,
                        robot_pose=self.current_pose,
                        human_pos=self.last_human_position
                    )
                    
                    # Always publish the avoidance command
                    self.wheel_speeds_pub.publish(cmd_vel)
                    
                    # Check for escape 
                    if should_escape:
                        self.fsm.trigger_event(NavigationEvent.ESCAPE_NEEDED)
                    
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
        """Publish human obstacle as PointCloud2 with direct coordinates"""
        if self.last_human_position is None:
            return
        
        try:
            # Create point cloud message with current timestamp
            # Using current time instead of trying to synchronize with transforms
            pc2 = PointCloud2()
            current_time = self.get_clock().now()
            pc2.header.stamp = current_time.to_msg()
            pc2.header.frame_id = "map"  # Use map frame directly to avoid transform issues
            
            # Define fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            pc2.fields = fields
            
            # Generate points - simplify to reduce processing load
            points = []
            human_x, human_y = self.last_human_position
            
            # Use fewer height levels and lower resolution to reduce processing
            height_levels = [0.1, 0.7, 1.4]  # Reduced height levels
            resolution = 0.05  # Reduced resolution
            
            for height in height_levels:
                for dx in np.arange(-radius, radius + resolution, resolution):
                    for dy in np.arange(-radius, radius + resolution, resolution):
                        dist_sq = dx*dx + dy*dy
                        if dist_sq <= radius*radius:
                            intensity = 254.0
                            points.append((human_x + dx, human_y + dy, height, intensity))
            
            # Pack point cloud
            pc2.height = 1
            pc2.width = len(points)
            pc2.point_step = 16
            pc2.row_step = pc2.point_step * pc2.width
            pc2.is_dense = True
            
            # Pack data
            point_data = bytearray()
            for p in points:
                point_data.extend(struct.pack('ffff', *p))
            pc2.data = point_data
            
            # Publish
            self.human_obstacles_pub.publish(pc2)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing human obstacle: {str(e)}')

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
