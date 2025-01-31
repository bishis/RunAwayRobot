#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import numpy as np
import math
from .processors.waypoint_generator import WaypointGenerator

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('robot_radius', 0.16)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('max_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.366)  # Actual max rotation speed
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
        
        # Publishers and subscribers
        self.wheel_speeds_pub = self.create_publisher(Twist, 'wheel_speeds', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 10)
        
        # Navigation action client - Add namespace to match Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')  # Add leading slash
        
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
        
        self.get_logger().info('Navigation controller initialized')

    def scan_callback(self, msg: LaserScan):
        """Store latest scan data"""
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        """Update map in waypoint generator"""
        self.waypoint_generator.update_map(msg)

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        try:
            # Log incoming command
            self.get_logger().info(
                f'Received cmd_vel - Linear: {msg.linear.x:.3f}, Angular: {msg.angular.z:.3f}'
            )

            # Handle small rotations differently
            if abs(msg.angular.z) > 0.0:
                # If we're trying to rotate, ensure minimum effective rotation
                if abs(msg.angular.z) < self.min_rotation_speed:
                    # Scale up to minimum rotation speed while preserving direction
                    msg.angular.z = math.copysign(self.min_rotation_speed, msg.angular.z)
                    # Stop linear motion during small rotations
                    msg.linear.x = 0.0
                    self.get_logger().info(
                        f'Small rotation detected, increasing to {msg.angular.z:.3f}'
                    )
            
            # Ensure we're not exceeding max speeds
            msg.linear.x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
            msg.angular.z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
            
            # Create and publish wheel speeds message
            wheel_speeds = Twist()
            wheel_speeds.linear.x = msg.linear.x
            wheel_speeds.angular.z = msg.angular.z
            self.wheel_speeds_pub.publish(wheel_speeds)
            
            # Log final speeds
            if abs(msg.linear.x) > 0.0 or abs(msg.angular.z) > 0.0:
                self.get_logger().info(
                    f'Publishing speeds - Linear: {msg.linear.x:.3f}, Angular: {msg.angular.z:.3f}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {str(e)}')
            # Stop robot on error
            stop_msg = Twist()
            self.wheel_speeds_pub.publish(stop_msg)

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
        """Main control loop for autonomous exploration"""
        try:
            # Only proceed if Nav2 is ready
            if not self.nav2_ready:
                return

            # If not navigating or previous goal failed, try to get new waypoint
            if not self.is_navigating or self.current_goal is None:
                self.get_logger().info('Generating new waypoint...')
                waypoint = self.waypoint_generator.generate_waypoint()
                if waypoint:
                    self.get_logger().info('Generated waypoint, sending goal...')
                    self.send_goal(waypoint)
                    # Publish visualization
                    markers = self.waypoint_generator.create_visualization_markers(waypoint)
                    self.marker_pub.publish(markers)
                else:
                    self.get_logger().warn('Failed to generate waypoint, will retry...')
            
            # Check for timeout on current goal
            if self.is_navigating and self.goal_start_time:
                time_elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
                if time_elapsed > self.goal_timeout:
                    self.get_logger().warn('Goal timeout reached, will try new waypoint')
                    self.handle_goal_failure()
        
        except Exception as e:
            self.get_logger().error(f'Error in exploration loop: {str(e)}')

    def send_goal(self, waypoint: PoseStamped):
        """Send navigation goal"""
        try:
            # Create and send goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = waypoint
            
            # Add debug info
            self.get_logger().info(
                f'Sending navigation goal:\n'
                f'  Position: ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})\n'
                f'  Frame: {waypoint.header.frame_id}\n'
                f'  Stamp: {waypoint.header.stamp.sec}.{waypoint.header.stamp.nanosec}'
            )
            
            # Send goal and register callbacks
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            # Add done callback
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.current_goal = waypoint
            self.is_navigating = True
            self.goal_start_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f'Error sending navigation goal: {str(e)}')
            self.handle_goal_failure()

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.handle_goal_failure()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation goal result"""
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
                self.consecutive_failures = 0
                self.is_navigating = False
                # Successfully reached goal, generate new waypoint
                self.exploration_loop()
            else:
                self.get_logger().warn(f'Navigation failed with status: {status}')
                self.handle_goal_failure()
        except Exception as e:
            self.get_logger().error(f'Error in goal result callback: {str(e)}')
            self.handle_goal_failure()

    def handle_goal_failure(self):
        """Handle navigation failures"""
        self.consecutive_failures += 1
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.get_logger().warn('Too many consecutive failures, waiting before continuing...')
            self.consecutive_failures = 0
            # Add a small delay before trying again
            self.create_timer(5.0, self.reset_navigation_state, oneshot=True)
        else:
            # Immediately try a new waypoint
            self.reset_navigation_state()

    def reset_navigation_state(self):
        """Reset navigation state and try new waypoint"""
        self.is_navigating = False
        self.current_goal = None
        self.goal_start_time = None
        # Force exploration loop to generate new waypoint
        self.exploration_loop()

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Log progress
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation feedback - Distance remaining: '
            f'{feedback.distance_remaining:.2f}m'
        )

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
