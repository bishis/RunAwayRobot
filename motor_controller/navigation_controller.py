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

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
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
        
        # Initialize waypoint generator
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
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3
        self.previous_waypoint = None

        # Add state for Nav2 readiness
        self.nav2_ready = False
        self.nav2_check_timer = self.create_timer(1.0, self.check_nav2_ready)
        
        # Create timer for exploration control
        self.create_timer(1.0, self.exploration_loop)
        
        # Add timeout parameters
        self.goal_timeout = 15.0  # Shorter timeout for unreachable goals
        self.planning_attempts = 0
        self.max_planning_attempts = 2  # Max attempts before giving up
        
        # Add timer to check goal progress
        self.goal_check_timer = self.create_timer(1.0, self.check_goal_progress)
        
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
        
        # Add human avoidance controller
        self.human_avoidance = HumanAvoidanceController(self, self.waypoint_generator)
        
        # Add escape-specific parameters
        self.escape_timeout = 45.0  # Longer timeout for escape attempts
        self.max_escape_attempts = 3  # Number of retry attempts for escape
        self.escape_attempts = 0  # Counter for escape attempts

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg
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

            # Check if map is complete
            if self.waypoint_generator.is_map_complete():
                self.get_logger().info('Map exploration complete!')
                # Optional: Stop the exploration loop
                # self.create_timer(1.0, self.exploration_loop).cancel()
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
            
            if status != GoalStatus.STATUS_SUCCEEDED:
                # Check for specific failure messages
                if hasattr(result, 'result') and hasattr(result.result, 'error_code'):
                    if result.result.error_code == 100:  # Nav2 error code for no valid path
                        self.get_logger().error('ROBOT IS TRAPPED - No valid escape path found!')
                
                self.get_logger().warn(f'Navigation failed with status: {status}')
                
                # Handle escape waypoint failures differently
                if self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
                    self.escape_attempts += 1
                    if self.escape_attempts < self.max_escape_attempts:
                        self.get_logger().warn(f'Retrying escape plan (attempt {self.escape_attempts + 1}/{self.max_escape_attempts})')
                        self.send_goal(self.current_goal)  # Retry same escape point
                        return
                    else:
                        self.get_logger().error('Max escape attempts reached, giving up escape plan')
                        self.escape_attempts = 0  # Reset for next time
                        self.get_logger().warn('Escape plan failed - resuming normal operation')
                
                # Normal failure handling
                self.consecutive_failures += 1
                self.planning_attempts += 1
                
                if self.planning_attempts >= self.max_planning_attempts:
                    self.get_logger().warn('Max planning attempts reached, forcing new waypoint')
                    self.planning_attempts = 0
                    self.waypoint_generator.force_waypoint_change()
                    
                if self.consecutive_failures >= self.max_consecutive_failures:
                    self.get_logger().warn('Too many consecutive failures, waiting before continuing...')
                    if not hasattr(self, '_reset_timer') or self._reset_timer is None:
                        self._reset_timer = self.create_timer(5.0, self.reset_with_timer)
                else:
                    self.reset_navigation_state()
            else:
                self.get_logger().info('Navigation succeeded')
                if self.current_goal is not None and self.is_escape_waypoint(self.current_goal):
                    self.get_logger().info('Escape plan succeeded - resuming normal operation')
                    self.escape_attempts = 0  # Reset escape attempts counter
                
                self.consecutive_failures = 0
                self.planning_attempts = 0
                self.reset_navigation_state()
                
        except Exception as e:
            self.get_logger().error(f'Error getting navigation result: {str(e)}')
            self.reset_navigation_state()

    def handle_goal_failure(self):
        """Handle navigation failures"""
        self.consecutive_failures += 1
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.get_logger().warn('Too many consecutive failures, waiting before continuing...')
            self.consecutive_failures = 0
            # Create a timer that will only fire once
            timer = self.create_timer(2.0, self.reset_with_timer)
            # Store timer to prevent garbage collection
            self._reset_timer = timer
        else:
            # Force waypoint generator to pick new point
            self.waypoint_generator.force_waypoint_change()
            self.reset_navigation_state()

    def reset_with_timer(self):
        """Callback for reset timer"""
        try:
            if hasattr(self, '_reset_timer') and self._reset_timer is not None:
                self._reset_timer.cancel()
                self._reset_timer = None
            # Reset navigation
            self.reset_navigation_state()
        except Exception as e:
            self.get_logger().error(f'Error in reset timer: {str(e)}')
            # Still try to reset navigation even if timer cleanup fails
            self.reset_navigation_state()

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
            self.get_logger().info('Canceling current navigation goal')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self._current_goal_handle = None

    def cancel_done_callback(self, future):
        """Handle goal cancellation result"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warn('Goal cancellation failed')

    def check_goal_progress(self):
        """Check if current goal is taking too long or stuck"""
        if not self.is_navigating or not self.goal_start_time:
            return
            
        # Check if we've exceeded the timeout
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
                    self.send_goal(self.current_goal)  # Retry same escape point
                    return
                else:
                    self.get_logger().error('Max escape attempts reached, giving up escape plan')
                    self.escape_attempts = 0  # Reset for next time
            
            # Normal waypoint handling
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
                    
                    # Only check for escape after publishing command
                    if needs_escape:
                        # Clear costmaps before planning escape
                        self.clear_costmaps()
                        
                        # Mark human position in costmap
                        self.mark_human_position()
                        
                        escape_point = self.human_avoidance.plan_escape()
                        if escape_point is not None:
                            self.send_goal(escape_point)
                            # Force tracking off when starting escape
                            self.is_tracking_human = False
                    
                self.get_logger().info(
                    f'Human tracking: dist={human_distance:.2f}m, '
                    f'backing_up={avoidance_cmd.linear.x:.2f}m/s'
                )
                
            except Exception as e:
                self.get_logger().error(f'Error in tracking cmd callback: {str(e)}')
                self.wheel_speeds_pub.publish(Twist())  # Stop on error

    def clear_costmaps(self):
        """Clear local and global costmaps"""
        try:
            # Create service client for clearing costmaps
            clear_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')
            while not clear_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for clear costmap service...')
            
            # Send request to clear costmaps
            clear_client.call_async(Empty.Request())
            self.get_logger().info('Cleared costmaps for escape planning')
            
        except Exception as e:
            self.get_logger().error(f'Failed to clear costmaps: {str(e)}')

    def mark_human_position(self):
        """Mark human position as obstacle in map and costmap"""
        try:
            # Subscribe to human coordinates
            human_pose = None
            human_sub = self.create_subscription(
                PoseStamped,
                '/human_coords',
                lambda msg: setattr(self, 'latest_human_pose', msg),
                1
            )
            
            # Wait briefly for human position
            rclpy.spin_once(self, timeout_sec=0.1)
            human_pose = getattr(self, 'latest_human_pose', None)
            
            if human_pose and self.current_map:
                # Create map update
                map_update = OccupancyGrid()
                map_update.header = self.current_map.header
                map_update.info = self.current_map.info
                map_update.data = list(self.current_map.data)  # Copy current map
                
                # Convert human position to map coordinates
                resolution = self.current_map.info.resolution
                origin_x = self.current_map.info.origin.position.x
                origin_y = self.current_map.info.origin.position.y
                
                # Calculate map indices for human position
                map_x = int((human_pose.pose.position.x - origin_x) / resolution)
                map_y = int((human_pose.pose.position.y - origin_y) / resolution)
                
                # Create a circular obstacle around human
                radius_cells = int(0.5 / resolution)  # 50cm radius
                for dx in range(-radius_cells, radius_cells + 1):
                    for dy in range(-radius_cells, radius_cells + 1):
                        # Check if point is within circle
                        if dx*dx + dy*dy <= radius_cells*radius_cells:
                            x = map_x + dx
                            y = map_y + dy
                            
                            # Ensure within map bounds
                            if (0 <= x < self.current_map.info.width and 
                                0 <= y < self.current_map.info.height):
                                # Calculate 1D index from 2D coordinates
                                idx = y * self.current_map.info.width + x
                                map_update.data[idx] = 100  # Mark as occupied
                
                # Create publisher for map updates
                map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
                
                # Publish updated map
                map_pub.publish(map_update)
                self.get_logger().info('Updated SLAM map with human position')
                
                # Clear costmaps to force update from new map
                self.clear_costmaps()
                
        except Exception as e:
            self.get_logger().error(f'Failed to mark human position: {str(e)}')

    def is_escape_waypoint(self, waypoint):
        """Check if waypoint is an escape waypoint"""
        return waypoint is not None and waypoint.header.stamp.nanosec == 1

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
