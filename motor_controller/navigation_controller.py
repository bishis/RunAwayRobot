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
        
        # Nav2 Service Clients
        self.get_state_client = {}
        for server in ['amcl', 'controller_server', 'planner_server', 'bt_navigator']:
            self.get_state_client[server] = self.create_client(
                GetState, f'/{server}/get_state')
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.waypoint_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')

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
        self.initial_pose_sent = True
        self.get_logger().info('Published initial pose')

    def check_nav2_servers_ready(self):
        """Check if Nav2 servers are ready."""
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

    def navigate_to_pose(self, goal_pose):
        """Navigate to a goal pose using Nav2."""
        if self.state not in [RobotState.IDLE, RobotState.NAVIGATING]:
            self.get_logger().warn('Cannot start navigation in current state')
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
        self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
        return True

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        if result.result == True:
            self.get_logger().info('Navigation succeeded')
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_next_waypoint()
            else:
                self.get_logger().info('All waypoints reached')
                self.state = RobotState.IDLE
        else:
            self.get_logger().error('Navigation failed')
            self.state = RobotState.RECOVERY

    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the sequence."""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().warn('No more waypoints to navigate to')
            self.state = RobotState.IDLE
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = self.waypoints[self.current_waypoint_index]
        
        # Calculate orientation towards the next waypoint or final orientation
        if self.current_waypoint_index < len(self.waypoints) - 1:
            next_point = self.waypoints[self.current_waypoint_index + 1]
            dx = next_point.x - goal_pose.pose.position.x
            dy = next_point.y - goal_pose.pose.position.y
            yaw = math.atan2(dy, dx)
        else:
            # Use current orientation for final waypoint
            yaw = 0.0
            
        # Convert yaw to quaternion
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        if not self.navigate_to_pose(goal_pose):
            self.get_logger().error(f'Failed to start navigation to waypoint {self.current_waypoint_index}')
            self.state = RobotState.RECOVERY

    def control_loop(self):
        """Main control loop."""
        if not self.current_pose or self.map_data is None:
            return

        if self.state == RobotState.INITIALIZING:
            if not self.initial_pose_sent:
                self.publish_initial_pose()
                self.state = RobotState.WAITING_FOR_NAV2
            return

        if self.state == RobotState.WAITING_FOR_NAV2:
            if self.check_nav2_servers_ready():
                self.state = RobotState.IDLE
                self.get_logger().info('Nav2 is ready')
            return

        self.publish_waypoints()

        if self.state == RobotState.IDLE:
            if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Generating new waypoints')
                self.waypoints = self.generate_waypoints()
                self.current_waypoint_index = 0
                if not self.waypoints:
                    self.get_logger().warn('Failed to generate waypoints')
                    return
            
            self.navigate_to_next_waypoint()

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
        
        check_radius = int((self.robot_radius + self.safety_margin) / self.map_info.resolution)
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
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