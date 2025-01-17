#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Point, Vector3, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Trigger
import math
from enum import Enum
import random
import numpy as np
from .processors.waypoint_generator import WaypointGenerator

class RobotState(Enum):
    INITIALIZING = 0
    WAITING_FOR_NAV2 = 1
    NAVIGATING = 2
    STOPPED = 3
    PLANNING = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Robot physical parameters
        self.declare_parameter('robot.radius', 0.17)
        self.declare_parameter('robot.safety_margin', 0.10)
        self.declare_parameter('waypoint_threshold', 0.2)
        self.declare_parameter('leg_length', 0.5)
        self.declare_parameter('num_waypoints', 8)
        
        # Get parameters
        self.robot_radius = self.get_parameter('robot.radius').value
        self.safety_margin = self.get_parameter('robot.safety_margin').value
        self.safety_radius = self.robot_radius + self.safety_margin
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.leg_length = self.get_parameter('leg_length').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        
        # Robot state
        self.state = RobotState.INITIALIZING
        self.current_pose = None
        self.latest_scan = None
        self.current_waypoint_index = 0
        self.waypoints = []
        self.current_path = None
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        
        # Nav2 Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # Nav2 Service Clients
        self.nav_through_poses_client = None
        self.clear_costmap_global_client = self.create_client(Trigger, 'clear_global_costmap')
        self.clear_costmap_local_client = self.create_client(Trigger, 'clear_local_costmap')
        
        # Lifecycle Service Clients
        self.get_state_client = {}
        for server in ['amcl', 'controller_server', 'planner_server', 'bt_navigator']:
            self.get_state_client[server] = self.create_client(
                GetState, f'/{server}/get_state')
        
        # Map data
        self.map_data = None
        self.map_info = None
        
        # Navigation tracking
        self.current_goal_handle = None
        self.is_navigating = False
        self.nav2_initialized = False
        self.initial_pose_sent = False
        
        # Timers
        self.create_timer(1.0, self.lifecycle_manager)
        self.create_timer(0.1, self.control_loop)
        
        # Waypoint generator
        self.waypoint_generator = WaypointGenerator(
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
            num_waypoints=self.num_waypoints
        )
        
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
                self.state = RobotState.PLANNING
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
        
        # Set initial covariance
        for i in range(36):
            msg.pose.covariance[i] = 0.0
        msg.pose.covariance[0] = 0.5   # x
        msg.pose.covariance[7] = 0.5   # y
        msg.pose.covariance[35] = 0.5  # yaw

        self.initial_pose_pub.publish(msg)
        self.get_logger().info('Published initial pose')

    def compute_path_to_pose(self, target_pose):
        """Request a path plan from Nav2."""
        if not self.nav2_initialized:
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = target_pose.x
        goal_msg.goal.pose.position.y = target_pose.y
        
        # Calculate goal orientation
        dx = target_pose.x - self.current_pose.position.x
        dy = target_pose.y - self.current_pose.position.y
        yaw = math.atan2(dy, dx)
        goal_msg.goal.pose.orientation.z = math.sin(yaw/2.0)
        goal_msg.goal.pose.orientation.w = math.cos(yaw/2.0)

        future = self.compute_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Path computation rejected')
            return None
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.status != 0:  # NavigationResult.STATUS_SUCCEEDED
            self.get_logger().error('Path computation failed')
            return None
            
        return result.result.path

    def navigate_to_pose(self, target_pose):
        """Send navigation goal to Nav2."""
        if not self.nav2_initialized:
            self.get_logger().warn('Nav2 not initialized yet')
            return False

        # First compute path
        path = self.compute_path_to_pose(target_pose)
        if path is None:
            self.get_logger().error('Failed to compute path')
            return False

        # If path is valid, send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_pose.x
        goal_msg.pose.pose.position.y = target_pose.y
        
        # Set orientation towards goal
        dx = target_pose.x - self.current_pose.position.x
        dy = target_pose.y - self.current_pose.position.y
        yaw = math.atan2(dy, dx)
        goal_msg.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw/2.0)

        try:
            self.current_goal_handle = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            self.current_goal_handle.add_done_callback(self.navigation_response_callback)
            self.is_navigating = True
            self.state = RobotState.NAVIGATING
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send navigation goal: {str(e)}')
            return False

    def path_callback(self, msg):
        """Handle planned path updates."""
        self.current_path = msg
        self.get_logger().debug('Received new path plan')

    def control_loop(self):
        """Main control loop using Nav2 for navigation."""
        if not self.current_pose or self.map_data is None:
            return

        if self.state == RobotState.INITIALIZING or self.state == RobotState.WAITING_FOR_NAV2:
            return

        self.publish_waypoints()

        if self.state == RobotState.PLANNING:
            if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Generating new waypoints')
                self.waypoints = self.generate_waypoints()
                self.current_waypoint_index = 0
                if not self.waypoints:
                    self.get_logger().warn('Failed to generate waypoints')
                    return
            
            current_target = self.waypoints[self.current_waypoint_index]
            if self.navigate_to_pose(current_target):
                self.state = RobotState.NAVIGATING
            else:
                self.get_logger().error('Failed to start navigation')

        elif self.state == RobotState.NAVIGATING:
            # Nav2 handles the navigation
            pass

    def navigation_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.is_navigating = False
            self.state = RobotState.PLANNING
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
            self.current_waypoint_index += 1
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            
        self.is_navigating = False
        self.state = RobotState.PLANNING

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    # Keep existing helper methods (generate_waypoints, publish_waypoints, etc.)
    # ... rest of the existing code ...

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