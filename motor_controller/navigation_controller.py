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
from .srv import HumanDetected

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('max_linear_speed', 0.07)
        self.declare_parameter('max_angular_speed', 0.9)  # Actual max rotation speed
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
        
        self.get_logger().info('Navigation controller initialized')
        
        self._current_goal_handle = None
        
        # Add human detection service
        self.human_service = self.create_service(
            HumanDetected,
            'human_detected',
            self.human_detected_callback
        )
        
        # Add tracking state
        self.tracking_human = False
        self.current_track_id = None
        self.last_human_pose = None

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        """Update map in waypoint generator and store locally"""
        self.current_map = msg  # Store map locally
        self.waypoint_generator.update_map(msg)

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        try:
            # Just handle rotation speeds
            if abs(msg.angular.z) > 0.0:
                if abs(msg.angular.z) < self.min_rotation_speed:
                    msg.angular.z = math.copysign(self.min_rotation_speed, msg.angular.z)
                    msg.linear.x = 0.0
            
            # Ensure we're not exceeding max speeds
            msg.linear.x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
            msg.angular.z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
            
            # Publish wheel speeds
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
        """Modified exploration loop to handle human tracking"""
        try:
            if self.tracking_human:
                # If tracking human, just keep facing them
                if self.last_human_pose is not None:
                    self.face_human(self.last_human_pose)
            else:
                # Normal exploration behavior
                waypoint = self.waypoint_generator.generate_waypoint()
                if waypoint:
                    # Additional safety check before sending
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
                        self.send_goal(waypoint)
                        markers = self.waypoint_generator.create_visualization_markers(waypoint)
                        self.marker_pub.publish(markers)
                    else:
                        self.get_logger().warn('Generated waypoint too close to wall, skipping')
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
            status = future.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'Navigation failed with status: {status}')
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
        
        if time_navigating > self.goal_timeout:
            self.get_logger().warn(f'Goal taking too long ({time_navigating:.1f}s), cancelling...')
            self.cancel_current_goal()
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

    def human_detected_callback(self, request, response):
        """Handle human detection"""
        try:
            # Stop exploration
            self.tracking_human = True
            self.current_track_id = request.track_id
            self.last_human_pose = request.human_pose
            
            # Cancel any current navigation goals
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
            
            # Turn to face human
            self.face_human(request.human_pose)
            
            response.success = True
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error handling human detection: {str(e)}')
            response.success = False
            return response
            
    def face_human(self, human_pose):
        """Turn robot to face human"""
        try:
            # Calculate angle to human
            dx = human_pose.pose.position.x - self.current_pose.pose.position.x
            dy = human_pose.pose.position.y - self.current_pose.pose.position.y
            target_angle = math.atan2(dy, dx)
            
            # Create Twist message to turn
            cmd = Twist()
            
            # Calculate angle difference
            current_angle = self.get_robot_yaw()
            angle_diff = target_angle - current_angle
            
            # Normalize angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            # Set angular velocity based on angle difference
            cmd.angular.z = max(min(angle_diff, 1.0), -1.0)
            
            # Publish command
            self.wheel_speeds_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error facing human: {str(e)}')

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
