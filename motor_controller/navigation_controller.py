#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from lifecycle_msgs.srv import GetState
import numpy as np
from enum import Enum, auto
import math
import time

class RobotState(Enum):
    INITIALIZING = auto()
    WAITING_FOR_NAV2 = auto()
    IDLE = auto()
    PLANNING = auto()
    NAVIGATING = auto()
    RECOVERY = auto()

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.robot_radius = 0.22  # meters
        self.safety_margin = 0.1  # meters
        self.nav2_timeout = 10.0  # seconds
        
        # State variables
        self.state = RobotState.INITIALIZING
        self.current_pose = None
        self.latest_scan = None
        self.map_data = None
        self.map_info = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.initial_pose_sent = False
        
        # Nav2 Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.waypoint_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Timer for checking Nav2 state
        self.create_timer(1.0, self.check_nav2_servers)
        
        self.get_logger().info('Navigation Controller initialized')

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

    def generate_waypoints(self, preferred_angle=None):
        """Generate waypoints using WaypointGenerator."""
        return self.waypoint_generator.generate_waypoints(
            self.current_pose,
            self.map_data,
            self.map_info,
            self.is_valid_point,
            self.map_to_world
        )

    def publish_waypoints(self):
        """Publish waypoints for visualization."""
        if not self.waypoints:
            return

        marker_array = MarkerArray()
        
        # All waypoints as red spheres
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = 'map'
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'waypoints'
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.POINTS
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        for point in self.waypoints:
            waypoint_marker.points.append(point)
        
        marker_array.markers.append(waypoint_marker)
        
        # Current target as green sphere
        if self.current_waypoint_index < len(self.waypoints):
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'current_target'
            target_marker.id = 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position = self.waypoints[self.current_waypoint_index]
            target_marker.pose.orientation.w = 1.0
            target_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(target_marker)
        
        self.waypoint_pub.publish(marker_array)

    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates."""
        if not self.map_info:
            return None, None
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return mx, my
        return None, None
        
    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates."""
        if not self.map_info:
            return None, None
        x = mx * self.map_info.resolution + self.map_info.origin.position.x
        y = my * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def is_valid_point(self, x, y):
        """Check if point is valid considering robot size."""
        mx, my = self.world_to_map(x, y)
        if mx is None or my is None:
            return False
        
        # Check area that covers entire robot plus safety margin
        check_radius = int((self.robot_radius + self.safety_margin) / self.map_info.resolution)
        
        # Check circular area around point
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                # Only check points within circular radius
                if dx*dx + dy*dy <= check_radius*check_radius:
                    check_x = mx + dx
                    check_y = my + dy
                    if (0 <= check_x < self.map_info.width and 
                        0 <= check_y < self.map_info.height):
                        if self.map_data[check_y, check_x] > 50:  # Occupied
                            return False
        return True

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