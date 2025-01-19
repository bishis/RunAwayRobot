#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import random
import math
import numpy as np
from enum import Enum
import time

class RobotState(Enum):
    INITIALIZING = 1
    NAVIGATING = 2
    WAITING_FOR_NAV2 = 3
    GENERATING_WAYPOINTS = 4

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Parameters
        self.declare_parameter('num_waypoints', 10)
        self.declare_parameter('min_waypoint_distance', 1.0)  # meters
        self.declare_parameter('max_waypoint_distance', 3.0)  # meters
        
        # Get parameters
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.min_distance = self.get_parameter('min_waypoint_distance').value
        self.max_distance = self.get_parameter('max_waypoint_distance').value
        
        # Navigation state
        self.state = RobotState.INITIALIZING
        self.current_waypoint_index = 0
        self.waypoints = []
        self.map_data = None
        self.map_info = None
        
        # Nav2 Action Client
        self.nav_client = None
        
        # Timer to initialize Nav2 client
        self.init_nav_client_timer = self.create_timer(1.0, self.init_nav_client)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            'map',      # SLAM Toolbox publishes /map
            self.map_callback,
            10
        )
        
        # Timer for the main control loop
        self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')

    def map_callback(self, msg):
        """Receives and stores the SLAM Toolbox occupancy grid."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
        if self.state == RobotState.INITIALIZING:
            self.state = RobotState.GENERATING_WAYPOINTS
            self.generate_waypoints()

    def generate_waypoints(self):
        """Generate random valid waypoints across the mapped area."""
        if self.map_data is None:
            self.get_logger().warn('No map data available yet; cannot generate waypoints.')
            return
        
        self.waypoints = []
        attempts = 0
        max_attempts = 1000
        
        # Find the bounds of the mapped area (exclude unknown=-1)
        mapped_points = np.where(self.map_data != -1)
        if len(mapped_points[0]) == 0:
            self.get_logger().warn('No known/mapped areas found in the map.')
            return

        min_y, max_y = np.min(mapped_points[0]), np.max(mapped_points[0])
        min_x, max_x = np.min(mapped_points[1]), np.max(mapped_points[1])
        
        # Convert to world coordinates
        world_min_x = min_x * self.map_info.resolution + self.map_info.origin.position.x
        world_max_x = max_x * self.map_info.resolution + self.map_info.origin.position.x
        world_min_y = min_y * self.map_info.resolution + self.map_info.origin.position.y
        world_max_y = max_y * self.map_info.resolution + self.map_info.origin.position.y
        
        self.get_logger().info(
            f'Generating waypoints in mapped area: '
            f'X: [{world_min_x:.2f}, {world_max_x:.2f}], '
            f'Y: [{world_min_y:.2f}, {world_max_y:.2f}]'
        )
        
        while len(self.waypoints) < self.num_waypoints and attempts < max_attempts:
            # Generate random point within mapped bounds
            x = random.uniform(world_min_x, world_max_x)
            y = random.uniform(world_min_y, world_max_y)
            
            # Check if this random point is valid and spaced from others
            if self.is_valid_point(x, y) and self.is_point_far_enough(x, y):
                self.waypoints.append(Point(x=x, y=y, z=0.0))
                self.get_logger().info(
                    f'Added waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})'
                )
            
            attempts += 1
        
        if not self.waypoints:
            self.get_logger().error('Failed to generate valid waypoints')
            return
            
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints')
        self.publish_waypoints()
        self.state = RobotState.NAVIGATING

    def is_point_far_enough(self, x, y):
        """Ensure the new waypoint is at least self.min_distance from existing ones."""
        for waypoint in self.waypoints:
            dx = x - waypoint.x
            dy = y - waypoint.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < self.min_distance:
                return False
        return True

    def is_valid_point(self, x, y):
        """Check if (x, y) is a free (unoccupied) cell in the occupancy grid."""
        if (self.map_data is None) or (self.map_info is None):
            return False
            
        # Convert world coords to map coords
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
            return False
        
        # Cell must be known (not -1) and free (<= 50 is typically free)
        if self.map_data[my, mx] == -1 or self.map_data[my, mx] > 50:
            return False
        
        # Optional: check a small neighborhood for safety
        radius = int(0.3 / self.map_info.resolution)  # 30cm safety
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                nx = mx + dx
                ny = my + dy
                if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                    if self.map_data[ny, nx] == -1 or self.map_data[ny, nx] > 50:
                        return False
        return True

    def publish_waypoints(self):
        """Publish all waypoints + current target for RViz visualization."""
        marker_array = MarkerArray()
        
        # All waypoints as red spheres
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = 'map'
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = 'waypoints'
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.2
        waypoint_marker.scale.y = 0.2
        waypoint_marker.scale.z = 0.2
        waypoint_marker.color.r = 1.0
        waypoint_marker.color.a = 1.0
        
        for p in self.waypoints:
            waypoint_marker.points.append(p)
        
        marker_array.markers.append(waypoint_marker)
        
        # Current target as green sphere
        if self.current_waypoint_index < len(self.waypoints):
            current_marker = Marker()
            current_marker.header.frame_id = 'map'
            current_marker.header.stamp = self.get_clock().now().to_msg()
            current_marker.ns = 'current_target'
            current_marker.id = 1
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            current_marker.pose.position = self.waypoints[self.current_waypoint_index]
            current_marker.scale.x = 0.3
            current_marker.scale.y = 0.3
            current_marker.scale.z = 0.3
            current_marker.color.g = 1.0
            current_marker.color.a = 1.0
            
            marker_array.markers.append(current_marker)
        
        self.waypoint_pub.publish(marker_array)

    def send_velocity_command(self, linear_x, angular_z):
        """
        Convert Nav2 velocity commands into a simple "binary" wheel command
        (Left, Right) for demonstration, then publish to /cmd_vel.
        """
        wheel_separation = 0.20  # meters
        wheel_radius = 0.05      # meters
        
        # Convert linear & angular to wheel speeds (rad/s)
        left_speed = (linear_x - (wheel_separation / 2.0) * angular_z) / wheel_radius
        right_speed = (linear_x + (wheel_separation / 2.0) * angular_z) / wheel_radius
        
        # Convert to discrete -1, 0, or +1
        def to_binary(speed):
            if speed > 0.1:
                return 1.0
            elif speed < -0.1:
                return -1.0
            return 0.0
        
        cmd = Twist()
        cmd.linear.x = to_binary(left_speed)
        cmd.angular.z = to_binary(right_speed)
        
        self.get_logger().info(
            f'Nav2 raw: lin={linear_x:.3f}, ang={angular_z:.3f} '
            f'=> left={left_speed:.2f}, right={right_speed:.2f} '
            f'=> cmd=({cmd.linear.x}, {cmd.angular.z})'
        )
        
        self.cmd_vel_pub.publish(cmd)

    def navigate_to_waypoint(self):
        """Send a new goal to /navigate_to_pose for the current waypoint."""
        if self.nav_client is None:
            self.get_logger().warn('Navigation client not initialized.')
            return False
        
        # Check server availability again
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Navigation server not available (timeout).')
            return False
        
        try:
            # Build the NavigateToPose Goal
            from nav2_msgs.action import NavigateToPose
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            current_waypoint = self.waypoints[self.current_waypoint_index]
            goal_msg.pose.pose.position = current_waypoint
            
            # Simple orientation: face toward the next waypoint (or 0 if last)
            if self.current_waypoint_index < len(self.waypoints) - 1:
                next_waypoint = self.waypoints[self.current_waypoint_index + 1]
                yaw = math.atan2(
                    next_waypoint.y - current_waypoint.y,
                    next_waypoint.x - current_waypoint.x
                )
            else:
                yaw = 0.0
            
            # Convert yaw -> quaternion
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self.get_logger().info(
                f'Sending goal for waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}: '
                f'({current_waypoint.x:.2f}, {current_waypoint.y:.2f})'
            )
            
            # Send asynchronously
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            send_goal_future.add_done_callback(self.navigation_response_callback)
            
            self.state = RobotState.WAITING_FOR_NAV2
            return True
        
        except Exception as e:
            self.get_logger().error(f'Failed to send navigation goal: {str(e)}')
            return False

    def navigation_response_callback(self, future):
        """Called when the action server has accepted/rejected our goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.state = RobotState.NAVIGATING
            return
        
        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Called when Nav2 finishes the goal."""
        result = future.result()
        status = result.status
        
        if status == 4:  # GoalStatus.SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints visited! Generating new set...')
                self.generate_waypoints()
            else:
                self.state = RobotState.NAVIGATING
        else:
            self.get_logger().warn(f'Navigation failed with status {status}')
            # Retry or move on
            self.state = RobotState.NAVIGATING

    def navigation_feedback_callback(self, feedback_msg):
        """Called periodically as Nav2 moves toward the goal."""
        feedback = feedback_msg.feedback
        current_vel = feedback.current_vel
        
        self.get_logger().info(
            f'Received Nav2 feedback: dist_rem={feedback.distance_remaining:.2f}, '
            f'lin_x={current_vel.linear.x:.2f}, ang_z={current_vel.angular.z:.2f}'
        )
        
        # Convert these velocities to the "binary" cmd_vel
        self.send_velocity_command(current_vel.linear.x, current_vel.angular.z)

    def control_loop(self):
        """Main control loop, runs at 1 Hz."""
        if self.state == RobotState.INITIALIZING:
            self.get_logger().info('Waiting for map data...')
            return
        
        elif self.state == RobotState.GENERATING_WAYPOINTS:
            self.get_logger().info('Generating waypoints...')
            return
        
        elif self.state == RobotState.NAVIGATING:
            # If we still have a waypoint to visit, send a goal
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_waypoint()
            else:
                self.get_logger().info('No more waypoints. Generating new ones...')
                self.generate_waypoints()
        
        elif self.state == RobotState.WAITING_FOR_NAV2:
            self.get_logger().info('Waiting for Nav2 feedback/result...')
        
        # Always publish markers
        self.publish_waypoints()

    def init_nav_client(self):
        """Initialize the action client for /navigate_to_pose."""
        if self.nav_client is None:
            self.get_logger().info('Creating /navigate_to_pose action client...')
            self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Check if server is available
        self.get_logger().info('Waiting for Nav2 action server...')
        server_available = self.nav_client.wait_for_server(timeout_sec=1.0)
        if server_available:
            self.get_logger().info('Nav2 action server is available!')
            self.destroy_timer(self.init_nav_client_timer)
        else:
            self.get_logger().warn('Nav2 server not ready yet. Will retry.')

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
