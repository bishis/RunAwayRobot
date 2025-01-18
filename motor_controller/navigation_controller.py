#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Vector3, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from lifecycle_msgs.srv import GetState
import math
from enum import Enum
import numpy as np

class RobotState(Enum):
    INITIALIZING = 1
    WAITING_FOR_NAV2 = 2
    IDLE = 3
    NAVIGATING = 4
    RECOVERY = 5

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Robot parameters
        self.declare_parameter('robot.radius', 0.17)
        self.declare_parameter('robot.safety_margin', 0.10)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot.radius').value
        self.safety_margin = self.get_parameter('robot.safety_margin').value
        
        # State variables
        self.state = RobotState.INITIALIZING
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.map_info = None
        self.nav2_initialized = False
        self.initial_pose_sent = False
        
        # Nav2 Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # Nav2 Service Clients
        self.get_state_client = {}
        for server in ['amcl', 'controller_server', 'planner_server', 'bt_navigator']:
            self.get_state_client[server] = self.create_client(
                GetState, f'/{server}/get_state')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.cmd_vel_callback, 10)
        
        # Timers
        self.create_timer(1.0, self.lifecycle_manager)
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')

    def lifecycle_manager(self):
        """Manage Nav2 initialization and lifecycle."""
        if self.state == RobotState.INITIALIZING:
            if self.current_pose is not None and not self.initial_pose_sent:
                self.publish_initial_pose()
                self.initial_pose_sent = True
                self.state = RobotState.WAITING_FOR_NAV2
                return
                
        elif self.state == RobotState.WAITING_FOR_NAV2:
            if self.check_nav2_servers_ready():
                self.nav2_initialized = True
                self.state = RobotState.IDLE
                self.get_logger().info('Nav2 is ready')
                return
            self.get_logger().warn('Waiting for Nav2 servers...')

    def check_nav2_servers_ready(self):
        """Check if all Nav2 servers are active."""
        if not all(self.get_state_client.values()):
            return False
            
        for server, client in self.get_state_client.items():
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{server} state service not available')
                return False
                
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is None or future.result().current_state.id != 3:
                self.get_logger().warn(f'{server} not active')
                return False
                
        return True

    def publish_initial_pose(self):
        """Publish the initial pose to Nav2."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self.current_pose
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Published initial pose')

    def navigate_to_pose(self, goal_pose):
        """Navigate to a goal pose using Nav2."""
        if not self.nav2_initialized:
            self.get_logger().warn('Nav2 not initialized yet')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending navigation goal')
        future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = RobotState.IDLE
            return False

        self.state = RobotState.NAVIGATING
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
        return True

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
            self.state = RobotState.IDLE
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.state = RobotState.RECOVERY

    def cmd_vel_callback(self, msg):
        """Convert Nav2 velocity commands to binary commands for the Pi."""
        try:
            # Convert to binary commands
            cmd = Twist()
            cmd.linear.x = 1.0 if msg.linear.x > 0.1 else (-1.0 if msg.linear.x < -0.1 else 0.0)
            cmd.angular.z = 1.0 if msg.angular.z > 0.1 else (-1.0 if msg.angular.z < -0.1 else 0.0)
            
            # Publish binary commands to the Pi
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().debug(
                f'Binary command sent - linear: {cmd.linear.x}, angular: {cmd.angular.z}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error converting velocity command: {str(e)}')

    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
        if self.state == RobotState.INITIALIZING and not self.initial_pose_sent:
            self.get_logger().info('Received initial odometry')

    def map_callback(self, msg):
        """Process incoming map data."""
        try:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_info = msg.info
            self.get_logger().debug('Map data updated')
        except Exception as e:
            self.get_logger().error(f'Error processing map data: {str(e)}')

    def compute_path(self, goal_pose):
        """Compute a path to the goal pose using Nav2."""
        if not self.nav2_initialized:
            self.get_logger().warn('Nav2 not initialized yet')
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose = self.current_pose

        self.get_logger().info('Computing path to goal')
        future = self.compute_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Path computation rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.path.poses:
            self.get_logger().error('No path found')
            return None

        return result.path

    def handle_recovery(self):
        """Handle recovery behavior when navigation fails."""
        if self.state != RobotState.RECOVERY:
            return

        self.get_logger().info('Executing recovery behavior')
        
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Wait a bit before retrying
        self.create_timer(2.0, self.retry_navigation, oneshot=True)

    def retry_navigation(self):
        """Retry navigation after recovery."""
        if self.state == RobotState.RECOVERY:
            self.state = RobotState.IDLE
            self.get_logger().info('Recovery complete, ready for new goals')

    def cancel_navigation(self):
        """Cancel the current navigation goal."""
        if self.state != RobotState.NAVIGATING:
            return

        self.get_logger().info('Canceling current navigation goal')
        
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Cancel the goal
        if self.nav_to_pose_client.server_is_ready():
            future = self.nav_to_pose_client._get_goal_handle()
            if future is not None:
                future.cancel_goal_async()
        
        self.state = RobotState.IDLE

    def control_loop(self):
        """Main control loop."""
        if not self.current_pose or self.map_data is None:
            return

        if self.state == RobotState.INITIALIZING or self.state == RobotState.WAITING_FOR_NAV2:
            self.lifecycle_manager()

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 