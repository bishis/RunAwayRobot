import math
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from scipy.ndimage import distance_transform_edt
import random
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class WaypointGenerator:
    """Generates exploration waypoints from occupancy grid maps"""
    
    def __init__(self, node: Node,
                 min_distance: float = 0.5,
                 safety_margin: float = 0.3,
                 waypoint_size: float = 0.3,
                 preferred_distance: float = 1.0,
                 goal_tolerance: float = 0.3):
        """
        Initialize waypoint generator.
        
        Args:
            node: ROS node for logging and TF
            min_distance: Minimum distance between waypoints
            safety_margin: Distance from walls
            waypoint_size: Size of waypoint markers
            preferred_distance: Preferred distance for new waypoints
            goal_tolerance: Distance to consider goal reached
        """
        self.node = node
        self.min_distance = min_distance
        self.safety_margin = safety_margin
        self.waypoint_size = waypoint_size
        self.preferred_distance = preferred_distance
        self.goal_tolerance = goal_tolerance
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        # State variables
        self.current_map = None
        self.current_waypoint = None
        self.last_waypoint_time = None
        self.min_waypoint_duration = 5.0  # Minimum time to keep a waypoint
        self.reached_waypoint = False

    def update_map(self, map_msg: OccupancyGrid):
        """Update stored map"""
        self.current_map = map_msg

    def is_valid_point(self, x: float, y: float) -> bool:
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

    def is_connected_to_robot(self, map_x: int, map_y: int, robot_x: int, robot_y: int, map_data: np.ndarray) -> bool:
        """Check if a point is connected to robot position without crossing walls"""
        # Use flood fill to check connectivity
        height, width = map_data.shape
        visited = np.zeros_like(map_data, dtype=bool)
        
        # Start flood fill from robot position
        queue = [(robot_x, robot_y)]
        visited[robot_y, robot_x] = True
        
        while queue:
            x, y = queue.pop(0)
            
            # Check if we've reached the target point
            if x == map_x and y == map_y:
                return True
            
            # Check all adjacent cells
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                next_x, next_y = x + dx, y + dy
                
                # Check bounds
                if (0 <= next_x < width and 0 <= next_y < height and
                    not visited[next_y, next_x] and
                    map_data[next_y, next_x] <= 50):  # Free or unknown space
                    
                    queue.append((next_x, next_y))
                    visited[next_y, next_x] = True
        
        return False

    def is_waypoint_valid(self) -> bool:
        """Check if current waypoint is still valid"""
        if not self.current_waypoint or not self.current_map:
            return False
            
        # Check if we've had this waypoint for minimum time
        if self.last_waypoint_time:
            time_since_waypoint = (self.node.get_clock().now() - self.last_waypoint_time).nanoseconds / 1e9
            if time_since_waypoint < self.min_waypoint_duration:
                return True

        # Check if point is still in free space
        x = self.current_waypoint.pose.position.x
        y = self.current_waypoint.pose.position.y
        return self.is_valid_point(x, y)

    def has_reached_waypoint(self) -> bool:
        """Check if robot has reached current waypoint (distance only, ignoring orientation)"""
        if not self.current_waypoint:
            return False
            
        try:
            # Get robot position
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Calculate distance to waypoint (only position, ignore orientation)
            dx = transform.transform.translation.x - self.current_waypoint.pose.position.x
            dy = transform.transform.translation.y - self.current_waypoint.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Only check distance to consider waypoint reached
            reached = distance < self.goal_tolerance
            if reached:
                self.node.get_logger().info(f'Reached waypoint (distance: {distance:.2f}m)')
                self.reached_waypoint = True
            
            return reached
            
        except TransformException:
            return False

    def generate_waypoint(self) -> PoseStamped:
        """Generate a single new waypoint prioritizing unexplored areas"""
        # Check if current waypoint is still valid and not reached
        if self.current_waypoint:
            if self.is_waypoint_valid() and not self.has_reached_waypoint():
                return self.current_waypoint
        
        if not self.current_map:
            return None
        
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Get map data as 2D array
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Create exploration potential field
        unknown_area = map_data == -1
        walls = map_data > 50
        
        # Calculate distance transform from walls
        wall_distance = distance_transform_edt(~walls) * resolution
        
        # Calculate distance to unknown areas
        unknown_distance = distance_transform_edt(~unknown_area) * resolution
        
        # Get current robot position
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convert to grid coordinates
            robot_grid_x = int((robot_x - origin_x) / resolution)
            robot_grid_y = int((robot_y - origin_y) / resolution)
        except TransformException:
            self.node.get_logger().warn('Could not get robot position')
            return None

        best_point = None
        best_score = -float('inf')
        
        # Sample points and score them
        for _ in range(50):  # Try 50 random points
            # Find valid cells (free space)
            valid_y, valid_x = np.where((map_data == 0) & (wall_distance > self.safety_margin))
            
            if len(valid_x) == 0:
                continue
            
            # Randomly select one of the valid cells
            idx = random.randint(0, len(valid_x) - 1)
            map_x = valid_x[idx]
            map_y = valid_y[idx]
            
            # Check if point is connected to robot position
            if not self.is_connected_to_robot(map_x, map_y, robot_grid_x, robot_grid_y, map_data):
                continue
            
            # Convert to world coordinates
            x = origin_x + map_x * resolution
            y = origin_y + map_y * resolution
            
            # Calculate scores for different criteria
            dist_to_robot = math.sqrt((map_x - robot_grid_x)**2 + (map_y - robot_grid_y)**2) * resolution
            if dist_to_robot < self.min_distance:
                continue
            
            # Score based on:
            # 1. Distance to unknown areas (prefer closer to unknown)
            unknown_score = 1.0 / (unknown_distance[map_y, map_x] + 0.1)
            
            # 2. Distance from walls (prefer points away from walls)
            wall_score = min(wall_distance[map_y, map_x], 2.0) / 2.0
            
            # 3. Appropriate distance from robot
            distance_score = 1.0 - abs(dist_to_robot - self.preferred_distance) / self.preferred_distance
            
            # Combine scores with weights
            total_score = (
                3.0 * unknown_score +  # Prioritize exploring unknown areas
                2.0 * wall_score +     # Prefer staying away from walls
                1.0 * distance_score   # Consider distance from robot
            )
            
            if total_score > best_score:
                best_score = total_score
                best_point = (x, y)
        
        if best_point is None:
            return None
        
        # Create waypoint with the best point
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.header.stamp = self.node.get_clock().now().to_msg()
        waypoint.pose.position.x = best_point[0]
        waypoint.pose.position.y = best_point[1]
        waypoint.pose.position.z = 0.0
        
        # Set orientation towards unexplored area
        # Find direction of closest unknown area
        unknown_y, unknown_x = np.where(unknown_area)
        if len(unknown_x) > 0:
            # Find closest unknown point
            dists = [(ux - map_x)**2 + (uy - map_y)**2 for ux, uy in zip(unknown_x, unknown_y)]
            closest_idx = np.argmin(dists)
            target_x = origin_x + unknown_x[closest_idx] * resolution
            target_y = origin_y + unknown_y[closest_idx] * resolution
            
            # Calculate angle towards unknown area
            angle = math.atan2(target_y - best_point[1], target_x - best_point[0])
            waypoint.pose.orientation.z = math.sin(angle / 2)
            waypoint.pose.orientation.w = math.cos(angle / 2)
        else:
            # If no unknown areas, just use default orientation
            waypoint.pose.orientation.w = 1.0
        
        # Store waypoint and time
        if best_point is not None:
            self.current_waypoint = waypoint
            self.last_waypoint_time = self.node.get_clock().now()
            self.reached_waypoint = False
            
        return waypoint

    def create_visualization_markers(self, waypoint: PoseStamped = None) -> MarkerArray:
        """Create visualization markers for waypoints"""
        markers = MarkerArray()
        
        if waypoint:
            # Create marker for waypoint
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = 0
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set marker position
            marker.pose = waypoint.pose
            
            # Set marker size
            marker.scale.x = self.waypoint_size
            marker.scale.y = self.waypoint_size
            marker.scale.z = 0.1
            
            # Set marker color (green)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            markers.markers.append(marker)
        
        return markers
