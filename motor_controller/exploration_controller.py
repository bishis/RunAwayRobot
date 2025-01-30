#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
import random

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Parameters
        self.declare_parameter('num_waypoints', 10)  # Number of waypoints to generate
        self.declare_parameter('min_distance', 0.5)  # Minimum distance between waypoints
        self.declare_parameter('goal_timeout', 30.0)  # Seconds before giving up on a goal
        self.declare_parameter('safety_margin', 0.3)  # Distance from walls
        self.declare_parameter('waypoint_size', 0.3)  # Size of waypoint markers
        self.declare_parameter('text_size', 0.25)     # Size of waypoint numbers
        
        # Get parameters
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.min_distance = self.get_parameter('min_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.waypoint_size = self.get_parameter('waypoint_size').value
        self.text_size = self.get_parameter('text_size').value
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            'map', 
            self.map_callback, 
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            'exploration_markers', 
            10
        )
        
        # State variables
        self.current_map = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        self.goal_start_time = None
        
        # Create timer for exploration control
        self.create_timer(1.0, self.exploration_loop)
        
        self.get_logger().info('Simple exploration controller initialized')

    def map_callback(self, msg):
        """Store map and generate waypoints if needed"""
        self.current_map = msg
        if not self.waypoints:
            self.generate_waypoints()

    def is_valid_point(self, x, y):
        """Check if a point is valid (free space and away from obstacles)"""
        if not self.current_map:
            return False
            
        # Convert world coordinates to map coordinates
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        
        # Check bounds
        if (map_x < 0 or map_x >= self.current_map.info.width or
            map_y < 0 or map_y >= self.current_map.info.height):
            return False
            
        # Get map data as 2D array
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Check if point and surrounding area is free space
        margin = int(self.safety_margin / resolution)
        for dy in range(-margin, margin + 1):
            for dx in range(-margin, margin + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                if (0 <= check_x < self.current_map.info.width and 
                    0 <= check_y < self.current_map.info.height):
                    if map_data[check_y, check_x] > 0:  # Not free space
                        return False
        return True

    def generate_waypoints(self):
        """Generate random waypoints in free space"""
        if not self.current_map:
            return
            
        self.waypoints = []
        attempts = 0
        max_attempts = 1000
        
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        while len(self.waypoints) < self.num_waypoints and attempts < max_attempts:
            # Generate random point in map bounds
            x = random.uniform(
                origin_x,
                origin_x + self.current_map.info.width * resolution
            )
            y = random.uniform(
                origin_y,
                origin_y + self.current_map.info.height * resolution
            )
            
            # Check if point is valid and far enough from other waypoints
            if self.is_valid_point(x, y):
                is_far_enough = True
                for wp in self.waypoints:
                    dist = math.sqrt((x - wp.pose.position.x)**2 + 
                                   (y - wp.pose.position.y)**2)
                    if dist < self.min_distance:
                        is_far_enough = False
                        break
                
                if is_far_enough:
                    # Create waypoint
                    waypoint = PoseStamped()
                    waypoint.header.frame_id = 'map'
                    waypoint.pose.position.x = x
                    waypoint.pose.position.y = y
                    waypoint.pose.position.z = 0.0
                    waypoint.pose.orientation.w = 1.0
                    self.waypoints.append(waypoint)
                    
                    self.get_logger().info(f'Generated waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})')
            
            attempts += 1
        
        self.publish_waypoint_markers()

    def publish_waypoint_markers(self):
        """Publish visualization markers for waypoints"""
        marker_array = MarkerArray()
        
        # First add a deletion marker to clear old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'waypoints'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, waypoint in enumerate(self.waypoints):
            # Sphere marker for waypoint position
            sphere_marker = Marker()
            sphere_marker.header.frame_id = 'map'
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'waypoints'
            sphere_marker.id = i * 2  # Use even numbers for spheres
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            sphere_marker.pose = waypoint.pose
            sphere_marker.scale.x = self.waypoint_size
            sphere_marker.scale.y = self.waypoint_size
            sphere_marker.scale.z = self.waypoint_size
            
            # Current waypoint is red, others are blue
            sphere_marker.color.a = 1.0
            if i == self.current_waypoint_index:
                sphere_marker.color.r = 1.0
                sphere_marker.color.g = 0.0
                sphere_marker.color.b = 0.0
            else:
                sphere_marker.color.r = 0.0
                sphere_marker.color.g = 0.0
                sphere_marker.color.b = 1.0
            
            # Text marker to show waypoint number
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoints'
            text_marker.id = i * 2 + 1  # Use odd numbers for text
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = waypoint.pose
            text_marker.pose.position.z += 0.3  # Place text above sphere
            text_marker.text = str(i + 1)  # Waypoint number
            
            text_marker.scale.z = self.text_size  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(sphere_marker)
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def exploration_loop(self):
        """Main control loop for exploration"""
        if not self.waypoints:
            return
            
        if not self.is_navigating:
            # Move to next waypoint
            goal = self.waypoints[self.current_waypoint_index]
            
            self.get_logger().info(
                f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
            )
            
            # Send goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal
            
            self.nav_client.wait_for_server()
            self.current_goal = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            self.current_goal.add_done_callback(self.goal_response_callback)
            
            self.is_navigating = True
            self.goal_start_time = self.get_clock().now()
            
        else:
            # Check for timeout
            if self.goal_start_time:
                time_elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
                if time_elapsed > self.goal_timeout:
                    self.get_logger().warn('Goal timeout reached, moving to next waypoint')
                    self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        """Advance to next waypoint"""
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        self.is_navigating = False
        self.publish_waypoint_markers()

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.move_to_next_waypoint()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation goal result"""
        self.move_to_next_waypoint()

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()