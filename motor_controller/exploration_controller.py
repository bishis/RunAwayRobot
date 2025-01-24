#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from .processors.waypoint_generator import WaypointGenerator
import numpy as np

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Create waypoint generator
        self.waypoint_generator = WaypointGenerator()
        
        # Subscribe to map updates
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        # Set up TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store latest map data
        self.current_map = None
        self.current_pose = None
        
        # Timer for periodic updates
        self.create_timer(1.0, self.update_exploration)  # Increased frequency to 1Hz
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'marker', 10)  # Changed topic to 'marker'
        
        # Publisher for navigation goals
        self.nav_goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.current_goal = None
        
        self.get_logger().info('Exploration Controller initialized')
        
    def map_callback(self, msg):
        """Store latest map data"""
        self.current_map = msg
        self.get_logger().debug('Received map update')
        
    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose: {ex}')
            return None
            
    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates"""
        if not self.current_map:
            return None, None
            
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        wx = origin_x + (mx * resolution)
        wy = origin_y + (my * resolution)
        return wx, wy
        
    def is_valid_point(self, x, y):
        """Check if a point is valid for navigation"""
        if not self.current_map:
            return False
            
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Convert world coordinates to map coordinates
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        
        # Check bounds
        if mx < 0 or mx >= self.current_map.info.width or \
           my < 0 or my >= self.current_map.info.height:
            return False
            
        # Check if point is in free space
        index = my * self.current_map.info.width + mx
        return self.current_map.data[index] == 0
        
    def update_exploration(self):
        """Generate and follow new waypoints"""
        if not self.current_map:
            self.get_logger().warning('No map data available')
            return
            
        current_pose = self.get_robot_pose()
        if not current_pose:
            self.get_logger().warning('No robot pose available')
            return
            
        # Convert map message to numpy array for processing
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Generate waypoints
        waypoints = self.waypoint_generator.generate_waypoints(
            current_pose.pose,
            map_data,
            self.current_map.info,
            self.is_valid_point,
            self.map_to_world
        )
        
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')
        
        # Clear old markers
        clear_markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = 'exploration_waypoints'
        clear_marker.action = Marker.DELETEALL
        clear_markers.markers.append(clear_marker)
        self.marker_pub.publish(clear_markers)
        
        # Publish visualization markers
        marker_array = MarkerArray()
        
        for i, point in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'exploration_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Alternate colors for frontier and boundary points
            if i % 2 == 0:  # Frontier points in blue
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            else:  # Boundary points in green
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            
            marker_array.markers.append(marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            self.get_logger().debug('Published visualization markers')
        
        # Send navigation goal if we have waypoints and no current goal
        if waypoints and not self.current_goal:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position = waypoints[0]  # Navigate to first waypoint
            goal.pose.orientation.w = 1.0  # Default orientation
            
            self.current_goal = goal
            self.nav_goal_pub.publish(goal)
            self.get_logger().info(f'Published navigation goal: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()