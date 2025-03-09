#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math, time, enum, struct
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.msg import Costmap
from builtin_interfaces.msg import Time
from tf2_ros import TransformException, Buffer, TransformListener
from .processors.navigation_fsm import NavigationState, NavigationEvent, NavigationFSM

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller_fsm')
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize current pose
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.w = 1.0
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_rotation_speed', 0.8)
        self.declare_parameter('goal_timeout', 30.0)
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_rotation_speed = self.get_parameter('min_rotation_speed').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        
        # Publishers and subscribers - INITIALIZE THESE BEFORE DEPENDENT COMPONENTS
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            'map', 
            self.map_callback, 
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 10)
        self.tracking_active_sub = self.create_subscription(Bool, '/human_tracking_active', self.tracking_active_callback, 10)
        self.tracking_cmd_sub = self.create_subscription(PoseStamped, '/human_coords', self.tracking_cmd_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        
        # Human obstacle publisher
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
        
        # Initialize waypoint generator AFTER tf setup
        from .processors.waypoint_generator import WaypointGenerator
        self.waypoint_generator = WaypointGenerator(
            node=self,
            min_distance=0.5,
            safety_margin=self.safety_margin,
            waypoint_size=0.3,
            preferred_distance=1.0,
            goal_tolerance=0.3
        )
        
        # Initialize human avoidance AFTER waypoint generator and publishers
        from .processors.human_avoidance_controller import HumanAvoidanceController
        self.human_avoidance = HumanAvoidanceController(self, self.waypoint_generator)
        
        # State variables
        self.current_map = None
        self.latest_scan = None
        self.current_goal = None
        self.current_goal_handle = None
        self.is_navigating = False
        self.goal_start_time = None
        self.planning_attempts = 0
        self.max_planning_attempts = 2
        self.escape_attempts = 0
        self.max_escape_attempts = 2
        self.previous_escape_waypoint_failed = False
        self.shake_timer = None
        self.turn_start_time = None
        self.last_position_check = None
        self.last_check_position = None
        self.stuck_threshold = 0.05
        self.stuck_timeout = 10.0
        self.last_human_position = None
        self.last_human_timestamp = None
        self.human_tracking_timeout = 3.0
        self.tf_timeout = 0.1
        self.tf_retry_count = 3
        self.tf_last_error_time = self.get_clock().now()
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('Waiting for navigation action server...')
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Still waiting for navigation action server...')
        self.get_logger().info('Navigation server connected!')
        
        # --- Setup the FSM ---
        callbacks = {
            "on_enter_initializing": self.on_enter_initializing,
            "on_update_initializing": self.on_update_initializing,
            "on_exit_initializing": self.on_exit_initializing,
            
            "on_enter_idle": self.on_enter_idle,
            "on_update_idle": self.on_update_idle,
            "on_exit_idle": self.on_exit_idle,
            
            "on_enter_exploring": self.on_enter_exploring,
            "on_update_exploring": self.on_update_exploring,
            "on_exit_exploring": self.on_exit_exploring,
            
            "on_enter_human_tracking": self.on_enter_human_tracking,
            "on_update_human_tracking": self.on_update_human_tracking,
            "on_exit_human_tracking": self.on_exit_human_tracking,
            
            "on_enter_escaping": self.on_enter_escaping,
            "on_update_escaping": self.on_update_escaping,
            "on_exit_escaping": self.on_exit_escaping,
            
            "on_enter_shake_defense": self.on_enter_shake_defense,
            "on_update_shake_defense": self.on_update_shake_defense,
            "on_exit_shake_defense": self.on_exit_shake_defense,
            
            "on_enter_post_escape": self.on_enter_post_escape,
            "on_update_post_escape": self.on_update_post_escape,
            "on_exit_post_escape": self.on_exit_post_escape,
            
            "on_enter_error": self.on_enter_error,
            "on_update_error": self.on_update_error,
            "on_exit_error": self.on_exit_error,
        }
        self.fsm = NavigationFSM(self, callbacks)
        # Timer to update the FSM periodically.
        self.fsm_update_timer = self.create_timer(0.1, self.fsm_update)
        
        self.get_logger().info('Navigation controller (FSM) initialized')
    
    # --- FSM Update ---
    def fsm_update(self):
        self.fsm.update()
    
    # --- Utility: Get current pose using TF ---
    def get_current_pose(self):
        try:
            for attempt in range(self.tf_retry_count):
                try:
                    current_time = self.get_clock().now()
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time(),  # Latest transform
                        timeout=rclpy.duration.Duration(seconds=self.tf_timeout * (attempt + 1))
                    )
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = current_time.to_msg()
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    self.current_pose = pose
                    return pose
                except TransformException:
                    if attempt == self.tf_retry_count - 1:
                        current_time = self.get_clock().now()
                        if (current_time - self.tf_last_error_time).nanoseconds / 1e9 > 5.0:
                            self.get_logger().warn(f'Transform lookup failed after {attempt+1} attempts.')
                            self.tf_last_error_time = current_time
                    continue
            return self.current_pose
        except Exception as e:
            current_time = self.get_clock().now()
            if (current_time - self.tf_last_error_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().error(f'Error getting current pose: {str(e)}')
                self.tf_last_error_time = current_time
            return self.current_pose
    
    # --- Subscribers Callbacks ---
    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.current_pose = self.get_current_pose()
        if hasattr(self, 'human_avoidance'):
            self.human_avoidance.latest_scan = msg
    
    def map_callback(self, msg: OccupancyGrid):
        """Debug map reception"""
        if self.current_map is None:
            self.get_logger().info(f"First map received! Size: {msg.info.width}x{msg.info.height}, Resolution: {msg.info.resolution}")
        else:
            self.get_logger().debug("Map update received")
        self.current_map = msg
        
        # Check if we're in EXPLORING state and were waiting for map
        if self.fsm.current_state == NavigationState.EXPLORING:
            self.get_logger().info("Map now available - retrying exploration")
            self.retry_exploration()
        self.waypoint_generator.update_map(msg)
    
    def cmd_vel_callback(self, msg: Twist):
        try:
            self.wheel_speeds_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {str(e)}')
            self.wheel_speeds_pub.publish(Twist())
    
    def tracking_active_callback(self, msg: Bool):
        """Fix the human tracking callback logic"""
        self.get_logger().info(f"Human tracking active: {msg.data}, current state: {self.fsm.current_state}")
        
        # For simplicity, trigger HUMAN_DETECTED/HUMAN_LOST events based on the flag.
        if self.fsm.current_state == NavigationState.ESCAPING:
            self.get_logger().info('Ignoring tracking during escape')
            return
        
        if msg.data:
            self.get_logger().info("Human detected - triggering event")
            self.fsm.trigger_event(NavigationEvent.HUMAN_DETECTED)
        else:
            # Only trigger HUMAN_LOST when in HUMAN_TRACKING state
            if self.fsm.current_state == NavigationState.HUMAN_TRACKING:
                self.get_logger().info("Human lost - triggering event")
                self.fsm.trigger_event(NavigationEvent.HUMAN_LOST)
    
    def tracking_cmd_callback(self, msg: PoseStamped):
        try:
            human_x = msg.pose.position.x
            human_y = msg.pose.position.y
            self.last_human_position = (human_x, human_y)
            self.last_human_timestamp = self.get_clock().now()
            if self.fsm.current_state == NavigationState.ESCAPING:
                return
            dx = human_x - self.current_pose.pose.position.x
            dy = human_y - self.current_pose.pose.position.y
            human_distance = math.sqrt(dx*dx + dy*dy)
            human_angle = math.atan2(dy, dx)
            self.get_logger().info(f'Human at ({human_x:.2f}, {human_y:.2f}), distance: {human_distance:.2f}m')
            # If human is too close, trigger escape.
            if self.fsm.current_state == NavigationState.HUMAN_TRACKING:
                cmd_vel, should_escape = self.human_avoidance.get_avoidance_command(
                    human_distance, human_angle,
                    robot_pose=self.current_pose,
                    human_pos=self.last_human_position
                )
                if should_escape:
                    self.get_logger().warn('Critical human distance – escape needed')
                    self.fsm.trigger_event(NavigationEvent.ESCAPE_NEEDED)
                self.wheel_speeds_pub.publish(cmd_vel)
        except Exception as e:
            self.get_logger().error(f'Error in tracking command callback: {str(e)}')
            self.wheel_speeds_pub.publish(Twist())
    
    # --- Navigation Goal Methods ---
    def send_goal(self, goal_msg: PoseStamped):
        try:
            self.cancel_current_goal()
            from nav2_msgs.action import NavigateToPose
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal_msg
            if self.is_escape_waypoint(goal_msg):
                self.get_logger().info('Escape goal detected – clearing emergency stop')
                time.sleep(0.5)
                self.wheel_speeds_pub.publish(Twist())
            self.get_logger().info(f"Sending goal at ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})")
            send_goal_future = self.nav_client.send_goal_async(
                nav_goal, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.current_goal = goal_msg
            self.is_navigating = True
            self.goal_start_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"Error sending goal: {str(e)}")
            self.reset_navigation_state()
    
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Goal rejected")
                if self.is_escape_waypoint(self.current_goal):
                    self.fsm.trigger_event(NavigationEvent.ESCAPE_FAILED)
                else:
                    self.fsm.trigger_event(NavigationEvent.GOAL_FAILED)
                return
            self.get_logger().info("Goal accepted")
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in goal response: {str(e)}")
            self.reset_navigation_state()
    
    def get_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f"Navigation result status: {status}")
            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.is_escape_waypoint(self.current_goal):
                    self.fsm.trigger_event(NavigationEvent.ESCAPE_SUCCEEDED)
                else:
                    self.fsm.trigger_event(NavigationEvent.GOAL_REACHED)
            else:
                if self.is_escape_waypoint(self.current_goal):
                    self.fsm.trigger_event(NavigationEvent.ESCAPE_FAILED)
                else:
                    self.fsm.trigger_event(NavigationEvent.GOAL_FAILED)
        except Exception as e:
            self.get_logger().error(f"Error getting navigation result: {str(e)}")
            self.reset_navigation_state()
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Navigation feedback - Distance remaining: '
            f'{feedback.distance_remaining:.2f}m'
        )
    
    def cancel_current_goal(self):
        try:
            if self.current_goal_handle is not None:
                self.clear_visualization_markers()
                if self.is_escape_waypoint(self.current_goal):
                    self.get_logger().info("Canceling escape goal")
                else:
                    self.get_logger().info("Canceling exploration goal")
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().error(f"Error canceling goal: {str(e)}")
                self.current_goal_handle = None
                self.current_goal = None
                self.is_navigating = False
                return True
            else:
                self.get_logger().info("No active goal to cancel")
                return False
        except Exception as e:
            self.get_logger().error(f"Error in cancel_current_goal: {str(e)}")
            self.current_goal_handle = None
            self.current_goal = None
            self.is_navigating = False
            return False
    
    def is_escape_waypoint(self, waypoint):
        # In this design, an escape waypoint is marked by setting its stamp.nanosec to 1.
        return waypoint is not None and waypoint.header.stamp.nanosec == 1
    
    def clear_visualization_markers(self):
        try:
            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)
            self.get_logger().debug("Cleared visualization markers")
        except Exception as e:
            self.get_logger().error(f"Error clearing markers: {str(e)}")
    
    def reset_navigation_state(self):
        self.current_goal = None
        self.is_navigating = False
        self.goal_start_time = None
        self.current_goal_handle = None
        self.last_position_check = None
        self.last_check_position = None
        self.planning_attempts = 0
    
    def publish_human_obstacle(self, radius=0.25):
        """Publish human obstacle as PointCloud2 with direct coordinates"""
        if self.last_human_position is None:
            return
        
        try:
            # Create point cloud message with current timestamp
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
    
    # --- FSM Callback Implementations ---
    # INITIALIZING
    def on_enter_initializing(self, event=None, data=None):
        self.get_logger().info("Entering INITIALIZING state")
    
    def on_update_initializing(self, event=None, data=None):
        """Fix the Nav2 ready check"""
        # Check if Nav2 server is ready.
        # The server_is_ready() method doesn't exist - use correct check
        if self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Nav2 action server is ready")
            self.fsm.trigger_event(NavigationEvent.NAV2_READY)
        else:
            self.get_logger().debug("Still waiting for Nav2 server to be ready...")
    
    def on_exit_initializing(self, event=None, data=None):
        self.get_logger().info("Exiting INITIALIZING state")
    
    # IDLE
    def on_enter_idle(self, event=None, data=None):
        """Add more debugging"""
        self.get_logger().info("Entering IDLE state")
        # For example, immediately request exploration.
        self.get_logger().info("Requesting exploration from IDLE state")
        self.fsm.trigger_event(NavigationEvent.EXPLORATION_REQUESTED)
    
    def on_update_idle(self, event=None, data=None):
        pass
    
    def on_exit_idle(self, event=None, data=None):
        self.get_logger().info("Exiting IDLE state")
    
    # EXPLORING
    def on_enter_exploring(self, event=None, data=None):
        """Add detailed debugging to waypoint generation"""
        self.get_logger().info("Entering EXPLORING state")
        
        # Check if map is available
        if self.current_map is None:
            self.get_logger().warn("No map available yet - waiting for map...")
            # Create a timer to retry after a delay
            self.create_timer(2.0, lambda: self.retry_exploration())
            return
        
        # Get current robot position before generating waypoint
        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().warn("Could not get robot position, using map center")
            # Will use map center as fallback
        
        self.get_logger().info("Generating exploration waypoint...")
        waypoint = self.waypoint_generator.generate_waypoint()
        
        if waypoint:
            self.get_logger().info(f"Generated waypoint at ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})")
            if self.current_map is not None and not self.waypoint_generator.is_near_wall(
                waypoint.pose.position.x,
                waypoint.pose.position.y,
                np.array(self.current_map.data).reshape(
                    self.current_map.info.height, self.current_map.info.width
                ),
                self.current_map.info.resolution,
                self.current_map.info.origin.position.x,
                self.current_map.info.origin.position.y
            ):
                self.current_goal = waypoint
                self.send_goal(waypoint)
                markers = self.waypoint_generator.create_visualization_markers(waypoint, is_escape=False)
                self.marker_pub.publish(markers)
            else:
                self.get_logger().warn("Generated waypoint too close to wall, forcing new one")
                self.waypoint_generator.force_waypoint_change()
                # Try again after a short delay
                self.create_timer(0.5, lambda: self.retry_exploration())
        else:
            self.get_logger().error("Failed to generate exploration waypoint")
            # Try again after a delay
            self.create_timer(2.0, lambda: self.retry_exploration())
    
    def on_update_exploring(self, event=None, data=None, state=None):
        if not self.is_navigating or self.current_goal is None:
            return
        current_time = self.get_clock().now()
        goal_duration = (current_time - self.goal_start_time).nanoseconds / 1e9 if self.goal_start_time else 0
        if goal_duration > self.goal_timeout:
            self.get_logger().warn(f"Exploration goal timeout: {goal_duration:.1f}s")
            self.fsm.trigger_event(NavigationEvent.GOAL_TIMEOUT)
        current_position = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        if self.last_position_check is None or self.last_check_position is None:
            self.last_position_check = current_time
            self.last_check_position = current_position
        time_diff = (current_time - self.last_position_check).nanoseconds / 1e9
        distance_moved = math.sqrt((current_position[0] - self.last_check_position[0])**2 +
                                   (current_position[1] - self.last_check_position[1])**2)
        if distance_moved < self.stuck_threshold and time_diff > self.stuck_timeout:
            self.get_logger().warn("Robot appears stuck during exploration")
            self.fsm.trigger_event(NavigationEvent.STUCK)
            self.last_position_check = None
            self.last_check_position = None
        elif distance_moved > self.stuck_threshold or time_diff > 10.0:
            self.last_position_check = current_time
            self.last_check_position = current_position
    
    def on_exit_exploring(self, event=None, data=None):
        self.get_logger().info("Exiting EXPLORING state")
        self.cancel_current_goal()
    
    # HUMAN_TRACKING
    def on_enter_human_tracking(self, event=None, data=None):
        self.get_logger().info("Entering HUMAN_TRACKING state")
        self.cancel_current_goal()
    
    def on_update_human_tracking(self, event=None, data=None, state=None):
        if self.last_human_timestamp is not None:
            current_time = self.get_clock().now()
            time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
            if time_since_human > self.human_tracking_timeout:
                self.fsm.trigger_event(NavigationEvent.HUMAN_LOST)
    
    def on_exit_human_tracking(self, event=None, data=None):
        self.get_logger().info("Exiting HUMAN_TRACKING state")
    
    # ESCAPING
    def on_enter_escaping(self, event=None, data=None):
        self.get_logger().info("Entering ESCAPING state")
        escape_point = self.human_avoidance.plan_escape(self.previous_escape_waypoint_failed)
        if escape_point:
            self.send_goal(escape_point)
        else:
            self.get_logger().error("Failed to plan escape point")
            self.fsm.trigger_event(NavigationEvent.ESCAPE_FAILED)
    
    def on_update_escaping(self, event=None, data=None, state=None):
        if not self.is_navigating or self.current_goal is None:
            return
        current_time = self.get_clock().now()
        goal_duration = (current_time - self.goal_start_time).nanoseconds / 1e9 if self.goal_start_time else 0
        if goal_duration > self.goal_timeout:
            self.get_logger().warn(f"Escape goal timeout: {goal_duration:.1f}s")
            self.fsm.trigger_event(NavigationEvent.GOAL_TIMEOUT)
        current_position = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        if self.last_position_check is None or self.last_check_position is None:
            self.last_position_check = current_time
            self.last_check_position = current_position
        time_diff = (current_time - self.last_position_check).nanoseconds / 1e9
        distance_moved = math.sqrt((current_position[0]-self.last_check_position[0])**2 +
                                   (current_position[1]-self.last_check_position[1])**2)
        if distance_moved < self.stuck_threshold and time_diff > self.stuck_timeout:
            self.get_logger().warn("Robot appears stuck during escaping")
            self.fsm.trigger_event(NavigationEvent.STUCK)
            self.last_position_check = None
            self.last_check_position = None
        elif distance_moved > self.stuck_threshold or time_diff > 10.0:
            self.last_position_check = current_time
            self.last_check_position = current_position
    
    def on_exit_escaping(self, event=None, data=None):
        self.get_logger().info("Exiting ESCAPING state")
        self.cancel_current_goal()
    
    # SHAKE_DEFENSE
    def on_enter_shake_defense(self, event=None, data=None):
        self.get_logger().warn("Entering SHAKE_DEFENSE state")
        self.cancel_current_goal()
        self.shake_count = 0
        self.shake_direction = 1
        if self.shake_timer:
            self.shake_timer.cancel()
        self.shake_timer = self.create_timer(0.2, self.execute_shake_motion)
    
    def on_update_shake_defense(self, event=None, data=None, state=None):
        current_time = self.get_clock().now()
        human_still_present = False
        if self.last_human_timestamp is not None:
            time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
            human_still_present = time_since_human < 3.0
        if not human_still_present:
            self.get_logger().info("Human no longer detected, stopping shake defense")
            self.wheel_speeds_pub.publish(Twist())
            if self.shake_timer:
                self.shake_timer.cancel()
                self.shake_timer = None
            self.fsm.trigger_event(NavigationEvent.RESUME)
    
    def on_exit_shake_defense(self, event=None, data=None):
        self.get_logger().info("Exiting SHAKE_DEFENSE state")
        if self.shake_timer:
            self.shake_timer.cancel()
            self.shake_timer = None
    
    # POST_ESCAPE
    def on_enter_post_escape(self, event=None, data=None):
        self.get_logger().info("Entering POST_ESCAPE state – turning to face human")
        self.turn_start_time = self.get_clock().now()

    def on_update_post_escape(self, event=None, data=None, state=None):
        if self.last_human_position is not None:
            dx = self.last_human_position[0] - self.current_pose.pose.position.x
            dy = self.last_human_position[1] - self.current_pose.pose.position.y
            target_angle = math.atan2(dy, dx)
            cmd = self.human_avoidance.turn_to_angle(target_angle)
            self.wheel_speeds_pub.publish(cmd)
            current_time = self.get_clock().now()
            turn_time = (current_time - self.turn_start_time).nanoseconds / 1e9
            if abs(cmd.angular.z) < 0.01 or turn_time > 10.0:
                self.get_logger().info("Turned to face human, resuming exploration")
                self.turn_start_time = None
                time.sleep(2)
                self.fsm.trigger_event(NavigationEvent.RESUME)
        else:
            self.get_logger().info("No known human position, resuming exploration")
            self.turn_start_time = None
            self.fsm.trigger_event(NavigationEvent.RESUME)
            
    def on_exit_post_escape(self, event=None, data=None):
        self.get_logger().info("Exiting POST_ESCAPE state")
        self.cancel_current_goal()
    
    # ERROR
    def on_enter_error(self, event=None, data=None):
        self.get_logger().error("Entering ERROR state")
    
    def on_update_error(self, event=None, data=None, state=None):
        pass
    
    def on_exit_error(self, event=None, data=None):
        self.get_logger().info("Exiting ERROR state")
    
    # --- Shake Motion Execution ---
    def execute_shake_motion(self):
        try:
            if self.fsm.current_state != NavigationState.SHAKE_DEFENSE:
                if self.shake_timer:
                    self.shake_timer.cancel()
                    self.shake_timer = None
                return
            current_time = self.get_clock().now()
            human_still_present = False
            if self.last_human_timestamp is not None:
                time_since_human = (current_time - self.last_human_timestamp).nanoseconds / 1e9
                human_still_present = time_since_human < 3.0
            if not human_still_present:
                self.get_logger().info("Human no longer detected during shake defense, stopping")
                self.wheel_speeds_pub.publish(Twist())
                if self.shake_timer:
                    self.shake_timer.cancel()
                    self.shake_timer = None
                self.fsm.trigger_event(NavigationEvent.RESUME)
                return
            cmd = Twist()
            if self.shake_count % 2 == 0:
                cmd.angular.z = 0.8 * self.shake_direction
            else:
                self.shake_direction *= -1
                cmd.angular.z = 0.8 * self.shake_direction
            self.wheel_speeds_pub.publish(cmd)
            self.get_logger().info(f"Shake motion: angular={cmd.angular.z:.2f}")
            self.shake_count += 1
        except Exception as e:
            self.get_logger().error(f"Error in shake motion: {str(e)}")
            self.wheel_speeds_pub.publish(Twist())
            if self.shake_timer:
                self.shake_timer.cancel()
                self.shake_timer = None
            self.fsm.trigger_event(NavigationEvent.ESCAPE_FAILED)

    def retry_exploration(self):
        """Retry exploration after a delay"""
        self.get_logger().info("Retrying exploration...")
        # Simulate a GOAL_TIMEOUT event to trigger new waypoint generation
        self.fsm.trigger_event(NavigationEvent.GOAL_TIMEOUT)

    def get_robot_position(self):
        """Get current robot position with better error handling"""
        try:
            # Try to get the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Update current pose from transform
            self.current_pose.header.stamp = self.get_clock().now().to_msg()
            self.current_pose.header.frame_id = 'map'
            self.current_pose.pose.position.x = transform.transform.translation.x
            self.current_pose.pose.position.y = transform.transform.translation.y
            self.current_pose.pose.position.z = transform.transform.translation.z
            self.current_pose.pose.orientation = transform.transform.rotation
            
            return (transform.transform.translation.x, transform.transform.translation.y)
        
        except TransformException as e:
            # Check if this is a repeated error
            current_time = self.get_clock().now()
            if (current_time - self.tf_last_error_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().warn(f"Could not get robot position: {str(e)}")
                self.tf_last_error_time = current_time
            
            # If SLAM isn't initialized yet, return a default position
            if self.current_map is not None:
                # Return the middle of the map as fallback position
                map_middle_x = self.current_map.info.origin.position.x + (self.current_map.info.width * self.current_map.info.resolution) / 2
                map_middle_y = self.current_map.info.origin.position.y + (self.current_map.info.height * self.current_map.info.resolution) / 2
                self.get_logger().warn(f"Using map center as fallback position: ({map_middle_x:.2f}, {map_middle_y:.2f})")
                return (map_middle_x, map_middle_y)
            return None

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