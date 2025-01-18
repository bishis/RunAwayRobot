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
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
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
            
        # Check if point and surrounding area is free
        radius = int(0.3 / self.map_info.resolution)  # 30cm safety radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_info.width and 
                    0 <= check_y < self.map_info.height):
                    if self.map_data[check_y, check_x] > 50:  # Occupied
                        return False
        return True

    def generate_waypoints(self):
        """Generate random valid waypoints across the map."""
        self.waypoints = []
        attempts = 0
        max_attempts = 1000
        
        while len(self.waypoints) < self.num_waypoints and attempts < max_attempts:
            # Generate random point in map
            x = random.uniform(
                self.map_info.origin.position.x,
                self.map_info.origin.position.x + self.map_info.width * self.map_info.resolution
            )
            y = random.uniform(
                self.map_info.origin.position.y,
                self.map_info.origin.position.y + self.map_info.height * self.map_info.resolution
            )
            
            # Check if point is valid and far enough from other waypoints
            if self.is_valid_point(x, y) and self.is_point_far_enough(x, y):
                self.waypoints.append(Point(x=x, y=y, z=0.0))
                self.get_logger().info(f'Added waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})')
            
            attempts += 1
        
        self.publish_waypoints()
        if self.waypoints:
            self.state = RobotState.NAVIGATING
        else:
            self.get_logger().error('Failed to generate valid waypoints')

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
        cmd = Twist()
        # Convert to binary values (-1, 0, 1)
        cmd.linear.x = 1.0 if linear_x > 0.1 else (-1.0 if linear_x < -0.1 else 0.0)
        cmd.angular.z = 1.0 if angular_z > 0.1 else (-1.0 if angular_z < -0.1 else 0.0)
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f'Sent command - linear: {cmd.linear.x}, angular: {cmd.angular.z}')

    def navigate_to_waypoint(self):
        """Send navigation goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

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
            yaw = 0.0  # Default orientation for final waypoint
            
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'({current_waypoint.x:.2f}, {current_waypoint.y:.2f})'
        )
        
        try:
            future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            future.add_done_callback(self.navigation_response_callback)
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

        self.get_logger().info('Navigation goal accepted')
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
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def control_loop(self):
        """Main control loop."""
        if self.state == RobotState.INITIALIZING:
            self.get_logger().info('Waiting for map data...')
            return
            
        elif self.state == RobotState.GENERATING_WAYPOINTS:
            self.get_logger().info('Generating waypoints...')
            return
            
        elif self.state == RobotState.NAVIGATING:
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_waypoint()
            else:
                self.generate_waypoints()
                
        elif self.state == RobotState.WAITING_FOR_NAV2:
            # Nav2 callbacks will handle state transitions
            pass
            
        # Always publish waypoints for visualization
        self.publish_waypoints()

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