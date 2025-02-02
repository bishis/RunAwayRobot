#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.ndimage import distance_transform_edt
from std_msgs.msg import ColorRGBA

class WaypointGenerator:
    """
    Generates exploration waypoints for autonomous robot navigation.
    Uses frontier detection and dynamic waypoint generation based on map updates.
    """
    
    def __init__(self, node: Node):
        """Initialize with parent node"""
        self.node = node  # Store node reference
        self.setup_parameters()
        self.setup_transforms()
        self.setup_state()
        self.setup_publishers()
        self.setup_subscribers()
        
    def setup_parameters(self):
        """Initialize all parameters"""
        # Navigation parameters
        self.min_distance = 0.5        # Minimum distance between waypoints
        self.safety_margin = 0.3       # Distance from walls
        self.waypoint_size = 0.3       # Size of waypoint markers
        self.preferred_distance = 1.0   # Preferred distance for new waypoints
        self.goal_tolerance = 0.5      # Distance to consider goal reached
        
        # Exploration parameters
        self.exploration_radius = 3.0   # Maximum exploration radius
        self.num_candidates = 16       # Number of candidate points to evaluate
        self.min_frontier_size = 5     # Minimum frontier size to consider
        
        # Scoring parameters
        self.min_score_threshold = 0.6
        self.score_improvement_threshold = 0.2
        self.distance_weight = 0.6
        self.frontier_weight = 0.4
        
        # Safety parameters
        self.wall_check_distance = 0.4
        self.min_valid_area = 1.0
        self.max_search_time = 1.0     # Maximum time to search for waypoints
        
        # Visualization parameters
        self.viz_update_rate = 0.5  # Update visualization every 0.5 seconds
        self.frontier_marker_size = 0.2
        self.candidate_marker_size = 0.15
        
    def setup_publishers(self):
        """Setup all publishers"""
        self.waypoint_pub = self.node.create_publisher(
            PoseStamped,
            'next_waypoint',
            10
        )
        self.viz_pub = self.node.create_publisher(
            MarkerArray,
            'waypoint_visualization',
            10
        )
        self.frontier_viz_pub = self.node.create_publisher(
            MarkerArray,
            'frontier_visualization',
            10
        )
        
        # Create timer for visualization updates
        self.viz_timer = self.node.create_timer(
            self.viz_update_rate,
            self.publish_visualization
        )
        
    def setup_subscribers(self):
        """Setup all subscribers"""
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            'map',
            self.update_map,
            10
        )
        
    def setup_transforms(self):
        """Initialize transform listener"""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
    def setup_state(self):
        """Initialize state variables"""
        self.current_map = None
        self.current_waypoint = None
        self.waypoint_cancelled = False
        self.force_new_waypoint = False
        self.position_history = []
        self.last_best_score = -float('inf')
        
    def update_map(self, map_msg: OccupancyGrid):
        """Process new map data and validate current waypoint"""
        self.current_map = map_msg
        if self.should_update_waypoint():
            self.force_new_waypoint = True
            
    def should_update_waypoint(self) -> bool:
        """Determine if current waypoint needs updating"""
        if not self.current_waypoint or not self.current_map:
            return False
            
        x = self.current_waypoint.pose.position.x
        y = self.current_waypoint.pose.position.y
        
        return (self.is_near_wall(x, y) or 
                not self.is_point_safe(x, y))
                
    def generate_waypoint(self) -> PoseStamped:
        """Generate next exploration waypoint"""
        if not self.should_generate_new_waypoint():
            return self.current_waypoint
            
        self.current_frontiers = self.find_frontiers()
        if not self.current_frontiers:
            return None
            
        self.current_candidates = self.generate_candidates(self.current_frontiers)
        best_waypoint = self.select_best_candidate(self.current_candidates)
        
        if best_waypoint:
            self.current_waypoint = self.create_pose_stamped(best_waypoint)
            self.waypoint_pub.publish(self.current_waypoint)
            
        return self.current_waypoint
        
    def find_frontiers(self):
        """Detect frontiers between explored and unexplored space"""
        if not self.current_map:
            return []
            
        map_data = self.get_map_data()
        known_space = map_data != -1
        free_space = map_data < 50
        
        # Find frontier boundaries
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(free_space.astype(np.uint8), kernel)
        frontier_candidates = dilated & ~known_space
        
        return self.process_frontier_candidates(frontier_candidates)
        
    def process_frontier_candidates(self, candidates):
        """Process and filter frontier candidates"""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            candidates.astype(np.uint8))
            
        frontiers = []
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] > self.min_frontier_size:
                frontiers.append({
                    'center': (centroids[i][0], centroids[i][1]),
                    'size': stats[i, cv2.CC_STAT_AREA]
                })
                
        return frontiers
        
    def generate_candidates(self, frontiers):
        """Generate and score waypoint candidates"""
        robot_pos = self.get_robot_position()
        if not robot_pos:
            return []
            
        candidates = []
        for frontier in frontiers:
            point = self.frontier_to_map_coords(frontier['center'])
            if self.is_valid_candidate(point, robot_pos):
                score = self.score_candidate(point, robot_pos, frontier['size'])
                candidates.append({'point': point, 'score': score})
                
        return candidates
        
    def score_candidate(self, point, robot_pos, frontier_size):
        """Score a waypoint candidate"""
        distance = self.calculate_distance(point, robot_pos)
        if distance > self.exploration_radius:
            return -float('inf')
            
        distance_score = 1.0 - (distance / self.exploration_radius)
        if distance < self.min_distance:
            distance_score *= 0.5
            
        size_score = min(frontier_size / 100.0, 1.0)
        
        return (self.distance_weight * distance_score + 
                self.frontier_weight * size_score)
                
    def create_visualization_markers(self) -> MarkerArray:
        """Create visualization markers for current waypoint"""
        markers = MarkerArray()
        
        if self.current_waypoint:
            marker = self.create_waypoint_marker()
            markers.markers.append(marker)
            
        return markers
        
    def create_waypoint_marker(self, waypoint) -> Marker:
        """Create marker for current waypoint"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose = waypoint.pose
        marker.scale.x = self.waypoint_size
        marker.scale.y = self.waypoint_size
        marker.scale.z = 0.3
        
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        return marker
        
    def create_frontier_marker(self, frontier, idx) -> Marker:
        """Create marker for frontier point"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Convert frontier center to map coordinates
        x, y = self.frontier_to_map_coords(frontier['center'])
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        
        # Size based on frontier size
        size = min(frontier['size'] / 100.0, 1.0) * self.frontier_marker_size
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        
        # Blue color with transparency
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.6)
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        return marker
        
    def create_candidate_marker(self, candidate, idx) -> Marker:
        """Create marker for candidate waypoint"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'candidates'
        marker.id = idx
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = candidate['point'][0]
        marker.pose.position.y = candidate['point'][1]
        marker.pose.position.z = 0.05
        
        marker.scale.x = self.candidate_marker_size
        marker.scale.y = self.candidate_marker_size
        marker.scale.z = 0.1
        
        # Color based on score (red to green)
        score = candidate['score']
        marker.color = ColorRGBA(
            r=1.0 - score,
            g=score,
            b=0.0,
            a=0.4
        )
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        return marker
        
    def delete_old_markers(self, *marker_arrays):
        """Send delete markers to clean up old visualizations"""
        for marker_array in marker_arrays:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            
    def frontier_to_map_coords(self, point):
        """Convert frontier coordinates to map coordinates"""
        x = point[0] * self.current_map.info.resolution + self.current_map.info.origin.position.x
        y = point[1] * self.current_map.info.resolution + self.current_map.info.origin.position.y
        return (x, y)

    # Helper methods
    def get_map_data(self):
        """Get map data as numpy array"""
        return np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
    def get_robot_position(self):
        """Get current robot position"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            return (transform.transform.translation.x,
                   transform.transform.translation.y)
        except TransformException:
            self.get_logger().warn('Could not get robot position')
            return None
            
    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between points"""
        return np.sqrt((point1[0] - point2[0])**2 + 
                      (point1[1] - point2[1])**2)

    def should_generate_new_waypoint(self) -> bool:
        """Determine if a new waypoint should be generated"""
        return self.force_new_waypoint or self.should_update_waypoint()

    def publish_visualization(self):
        """Publish all visualization markers"""
        # Create marker arrays
        waypoint_markers = MarkerArray()
        frontier_markers = MarkerArray()
        candidate_markers = MarkerArray()
        
        # Add current waypoint
        if self.current_waypoint:
            waypoint_markers.markers.append(
                self.create_waypoint_marker(self.current_waypoint)
            )
        
        # Add frontiers
        if hasattr(self, 'current_frontiers'):
            for idx, frontier in enumerate(self.current_frontiers):
                frontier_markers.markers.append(
                    self.create_frontier_marker(frontier, idx)
                )
        
        # Add candidates
        if hasattr(self, 'current_candidates'):
            for idx, candidate in enumerate(self.current_candidates):
                candidate_markers.markers.append(
                    self.create_candidate_marker(candidate, idx)
                )
        
        # Delete old markers
        self.delete_old_markers(waypoint_markers, frontier_markers, candidate_markers)
        
        # Publish all markers
        self.viz_pub.publish(waypoint_markers)
        self.frontier_viz_pub.publish(frontier_markers)

    # Update logging calls
    def get_logger(self):
        return self.node.get_logger()

    def get_clock(self):
        return self.node.get_clock()
