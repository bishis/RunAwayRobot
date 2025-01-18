#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import random
import math
import numpy as np
from enum import Enum

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
        self.navigation_succeeded = False
        
        # Nav2 Action Client with retry
        self.nav_client = None
        self.create_timer(1.0, self.init_nav_client)  # Try to connect every second
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoints', 10)
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        # Timer for main control loop
        self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info('Navigation controller initialized')

    def map_callback(self, msg):
        """Process incoming map data."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
        if self.state == RobotState.INITIALIZING:
            self.state = RobotState.GENERATING_WAYPOINTS
            self.generate_waypoints()

    def is_valid_point(self, x, y):
        """Check if point is valid in map."""
        if self.map_data is None or self.map_info is None:
            return False
            
        # Convert world coordinates to map coordinates
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        # Check bounds
        if not (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height):
            return False
            
        # First check if the point is in known space (not -1/unknown)
        if self.map_data[my, mx] == -1:  # Unknown space
            return False
            
        # Check if point and surrounding area is free and mapped
        radius = int(0.3 / self.map_info.resolution)  # 30cm safety radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_info.width and 
                    0 <= check_y < self.map_info.height):
                    cell_value = self.map_data[check_y, check_x]
                    # Check if cell is unknown or occupied
                    if cell_value == -1 or cell_value > 50:
                        return False
        return True

    def generate_waypoints(self):
        """Generate random valid waypoints across the mapped area."""
        self.waypoints = []
        attempts = 0
        max_attempts = 1000
        
        # Find the bounds of the mapped area
        mapped_points = np.where(self.map_data != -1)
        if len(mapped_points[0]) == 0:
            self.get_logger().warn('No mapped areas found')
            return
            
        # Get the bounds of the mapped area
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
            
            # Check if point is valid and far enough from other waypoints
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
        """Check if point is far enough from existing waypoints."""
        for waypoint in self.waypoints:
            dx = x - waypoint.x
            dy = y - waypoint.y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < self.min_distance:
                return False
        return True

    def publish_waypoints(self):
        """Publish waypoints for visualization."""
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
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color.g = 1.0
            target_marker.color.a = 1.0
            
            marker_array.markers.append(target_marker)
        
        self.waypoint_pub.publish(marker_array)

    def send_velocity_command(self, linear_x, angular_z):
        """Send binary velocity command to the robot."""
        # Calculate wheel speeds based on differential drive kinematics
        wheel_separation = 0.20  # Distance between wheels in meters
        wheel_radius = 0.05    # Wheel radius in meters
        
        # Convert linear and angular velocities to wheel speeds
        left_speed = (linear_x - (wheel_separation / 2) * angular_z) / wheel_radius
        right_speed = (linear_x + (wheel_separation / 2) * angular_z) / wheel_radius
        
        # Debug print raw speeds
        self.get_logger().info(
            f'\nRaw velocities:'
            f'\n  Linear X: {linear_x:.3f} m/s'
            f'\n  Angular Z: {angular_z:.3f} rad/s'
            f'\nCalculated wheel speeds:'
            f'\n  Left: {left_speed:.3f} rad/s'
            f'\n  Right: {right_speed:.3f} rad/s'
        )
        
        # Convert to binary commands (-1, 0, 1)
        binary_left = 1.0 if left_speed > 0.1 else (-1.0 if left_speed < -0.1 else 0.0)
        binary_right = 1.0 if right_speed > 0.1 else (-1.0 if right_speed < -0.1 else 0.0)
        
        # Create Twist message with binary wheel speeds
        cmd = Twist()
        cmd.linear.x = binary_left   # Left wheel
        cmd.angular.z = binary_right # Right wheel
        
        # Log the final commands being sent
        self.get_logger().info(
            f'Publishing cmd_vel:'
            f'\n  cmd_vel.linear.x (LEFT): {cmd.linear.x}'
            f'\n  cmd_vel.angular.z (RIGHT): {cmd.angular.z}'
        )
        
        self.cmd_vel_pub.publish(cmd)

    def navigate_to_waypoint(self):
        """Send navigation goal to Nav2."""
        if self.nav_client is None:
            self.get_logger().warn('Navigation client not initialized')
            return False
            
        # Double check server is still available
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation server not available, reinitializing...')
            self.nav_client = None
            self.create_timer(1.0, self.init_nav_client)
            return False

        # Debug print
        self.get_logger().info('Preparing to send navigation goal...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        current_waypoint = self.waypoints[self.current_waypoint_index]
        goal_msg.pose.pose.position = current_waypoint
        
        # Set orientation to face the direction of movement
        if self.current_waypoint_index < len(self.waypoints) - 1:
            next_waypoint = self.waypoints[self.current_waypoint_index + 1]
            yaw = math.atan2(
                next_waypoint.y - current_waypoint.y,
                next_waypoint.x - current_waypoint.x
            )
        else:
            yaw = 0.0
            
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending navigation goal for waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'({current_waypoint.x:.2f}, {current_waypoint.y:.2f})'
        )
        
        try:
            # Send goal with explicit feedback callback
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            self.get_logger().info('Navigation goal sent, waiting for response...')
            send_goal_future.add_done_callback(self.navigation_response_callback)
            self.state = RobotState.WAITING_FOR_NAV2
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send navigation goal: {str(e)}')
            return False

    def navigation_response_callback(self, future):
        """Handle the response from Nav2 goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = RobotState.NAVIGATING
            return

        self.get_logger().info('Navigation goal accepted, waiting for feedback...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle the result of navigation."""
        status = future.result().status
        if status == 4:  # Succeeded
            self.get_logger().info('Navigation succeeded')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints visited!')
                self.generate_waypoints()  # Generate new waypoints
            else:
                self.state = RobotState.NAVIGATING
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.state = RobotState.NAVIGATING

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback from Nav2."""
        self.get_logger().info('Received navigation feedback!')  # Debug print
        
        feedback = feedback_msg.feedback
        
        # Get current commanded velocities
        current_vel = feedback.current_vel
        linear_x = current_vel.linear.x
        angular_z = current_vel.angular.z
        
        # Log the raw Nav2 velocities
        self.get_logger().info(
            f'\nReceived Nav2 velocities:'
            f'\n  Linear X: {linear_x:.3f} m/s'
            f'\n  Angular Z: {angular_z:.3f} rad/s'
            f'\nDistance remaining: {feedback.distance_remaining:.2f}m'
        )
        
        # Send the binary velocity command
        self.send_velocity_command(linear_x, angular_z)

    def control_loop(self):
        """Main control loop."""
        if self.state == RobotState.INITIALIZING:
            self.get_logger().info('Waiting for map data...')
            return
            
        elif self.state == RobotState.GENERATING_WAYPOINTS:
            self.get_logger().info('Generating waypoints...')
            return
            
        elif self.state == RobotState.NAVIGATING:
            # Add diagnostic info
            self.get_logger().info(
                f'\nNavigation Status:'
                f'\n  Current State: {self.state}'
                f'\n  Nav Client Ready: {self.nav_client is not None}'
                f'\n  Waypoints Generated: {len(self.waypoints)}'
                f'\n  Current Waypoint: {self.current_waypoint_index}/{len(self.waypoints)}'
            )
            
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_waypoint()
            else:
                self.generate_waypoints()
                
        elif self.state == RobotState.WAITING_FOR_NAV2:
            self.get_logger().info('Waiting for Nav2 to process goal...')
            
        # Always publish waypoints for visualization
        self.publish_waypoints()

    def init_nav_client(self):
        """Initialize Nav2 action client with retry."""
        if self.nav_client is None:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            
        # Check if the action server is available
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Nav2 action server...')
            return
            
        # Check if the required transforms are available
        try:
            from tf2_ros import Buffer, TransformListener
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, self)
            
            # Debug transform tree
            frames = tf_buffer.all_frames_as_string()
            self.get_logger().info(f'Available transforms:\n{frames}')
            
            # Wait for the transform between map and base_link with timeout
            try:
                transform = tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0)
                )
                self.get_logger().info('Transform found between map and base_link')
            except Exception as e:
                self.get_logger().warn(f'Transform not ready: {str(e)}')
                return
                
        except Exception as e:
            self.get_logger().warn(f'Transform setup error: {str(e)}')
            return
            
        self.get_logger().info('Successfully connected to Nav2 action server')
        self.destroy_timer(self.init_nav_client)  # Stop retry timer

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