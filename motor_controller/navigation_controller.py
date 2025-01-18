#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Point, Vector3, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav2_msgs.action import NavigateToPose
import math
from enum import Enum
import random
import numpy as np
from .processors.path_planner import PathPlanner
from .processors.waypoint_generator import WaypointGenerator

class RobotState(Enum):
    INITIALIZING = 1
    NAVIGATING = 2
    STOPPED = 3
    PLANNING = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Robot physical parameters
        self.declare_parameter('robot.radius', 0.17)
        self.declare_parameter('robot.safety_margin', 0.10)
        
        # Navigation parameters
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
        self.initial_pose_received = False
        self.navigation_succeeded = True
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')
        
        # Add map subscriber
        self.map_data = None
        self.map_info = None
        
        # Keep path planner for compatibility
        self.path_planner = PathPlanner(
            safety_radius=self.safety_radius,
            num_samples=20,
            step_size=0.3
        )
        
        # Add loop closure parameters
        self.visited_positions = []
        self.loop_closure_threshold = 0.5
        self.min_travel_distance = 2.0
        self.last_position = None
        self.distance_traveled = 0.0
        
        self.waypoint_generator = WaypointGenerator(
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
            num_waypoints=self.num_waypoints
        )

    def navigate_to_pose(self, goal_pose):
        """Send navigation goal to Nav2."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = goal_pose
        
        # Calculate orientation towards goal
        if self.current_waypoint_index < len(self.waypoints) - 1:
            next_point = self.waypoints[self.current_waypoint_index + 1]
            dx = next_point.x - goal_pose.x
            dy = next_point.y - goal_pose.y
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0
            
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('Sending goal to Nav2')
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_response_callback)
        return True

    def navigation_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.navigation_succeeded = False
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
            self.navigation_succeeded = True
            self.current_waypoint_index += 1
            self.state = RobotState.PLANNING
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.navigation_succeeded = False
            self.state = RobotState.PLANNING

    def control_loop(self):
        """Main control loop using Nav2 for navigation."""
        if not self.current_pose or self.map_data is None:
            return

        self.publish_waypoints()

        if self.state == RobotState.INITIALIZING:
            if self.initial_pose_received:
                self.state = RobotState.PLANNING
            return

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

    # Keep existing methods but replace implementation with pass where not needed
    def check_obstacles(self):
        pass

    def determine_movement(self, sector_data):
        pass

    def send_velocity_command(self, linear_x, angular_z):
        pass

    def find_clear_path(self):
        pass

    def measure_gap(self, center_angle):
        pass

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

    def scan_callback(self, msg):
        """Handle LIDAR scan updates."""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
        
        # Generate initial waypoints if needed
        if not self.initial_pose_received and self.map_info is not None:
            self.initial_pose_received = True
            self.waypoints = self.generate_waypoints()

    def map_callback(self, msg):
        """Process incoming map data."""
        try:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_info = msg.info
            self.path_planner.update_map(self.map_data, self.map_info)
            self.get_logger().debug('Map data updated')
        except Exception as e:
            self.get_logger().error(f'Error processing map data: {str(e)}')

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
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 