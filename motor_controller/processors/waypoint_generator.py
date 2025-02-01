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
                 goal_tolerance: float = 0.5):
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
        self.force_new_waypoint = False
        self.reached_waypoint = False
        
        # Add stability parameters
        self.min_score_threshold = 0.6  # Minimum score to consider a new waypoint
        self.score_improvement_threshold = 0.2  # Required improvement to switch waypoints
        self.last_best_score = -float('inf')
        self.waypoint_attempts = 0
        self.max_attempts = 3  # Maximum attempts before accepting a lower score
        self.wall_check_distance = 0.4  # Distance to check for walls
        self.max_wall_retry = 5  # Maximum attempts to find non-wall waypoint

    def update_map(self, map_msg: OccupancyGrid):
        """Update stored map and validate current waypoint"""
        self.current_map = map_msg
        
        # Check if current waypoint is too close to walls
        if self.current_waypoint and self.current_map:
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            x = self.current_waypoint.pose.position.x
            y = self.current_waypoint.pose.position.y
            
            if self.is_near_wall(x, y, map_data, 
                               self.current_map.info.resolution,
                               self.current_map.info.origin.position.x,
                               self.current_map.info.origin.position.y):
                self.node.get_logger().warn('Current waypoint is too close to wall, forcing new waypoint')
                self.force_waypoint_change()
                return
            
            # Also check if point is still in free space
            if not self.is_valid_point(x, y):
                self.node.get_logger().warn('Current waypoint is no longer in free space, forcing new waypoint')
                self.force_waypoint_change()

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
            
        # Only check if point is still in free space
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

    def force_waypoint_change(self):
        """Force the generator to pick a new waypoint"""
        self.force_new_waypoint = True
        self.current_waypoint = None

    def is_near_wall(self, x: float, y: float, costmap_data: np.ndarray, resolution: float, origin_x: float, origin_y: float) -> bool:
        """Check if a point is too close to walls in multiple directions"""
        # Convert to map coordinates
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        
        # Check points in a circle around the position
        check_radius = int(self.wall_check_distance / resolution)
        angles = np.linspace(0, 2*np.pi, 16)  # Check 16 directions
        
        for angle in angles:
            # Get point at check_radius distance in this direction
            check_x = int(map_x + check_radius * np.cos(angle))
            check_y = int(map_y + check_radius * np.sin(angle))
            
            # Ensure within bounds
            if (0 <= check_x < costmap_data.shape[1] and 
                0 <= check_y < costmap_data.shape[0]):
                # Check if there's a wall (cost > 50 typically indicates obstacle)
                if costmap_data[check_y, check_x] > 50:
                    return True
        return False

    def find_alternative_waypoint(self, x: float, y: float, costmap_data: np.ndarray, 
                                resolution: float, origin_x: float, origin_y: float) -> tuple[float, float]:
        """Find alternative waypoint away from walls"""
        search_radius = 0.5  # meters
        angles = np.linspace(0, 2*np.pi, 8)  # Try 8 different directions
        
        for radius in np.linspace(0.2, search_radius, 3):
            for angle in angles:
                new_x = x + radius * np.cos(angle)
                new_y = y + radius * np.sin(angle)
                
                if not self.is_near_wall(new_x, new_y, costmap_data, resolution, origin_x, origin_y):
                    return new_x, new_y
        
        return x, y  # Return original if no alternative found

    def generate_waypoint(self) -> PoseStamped:
        """Generate a single new waypoint prioritizing unexplored areas"""
        # Keep current waypoint unless forced to change or reached
        if self.current_waypoint and not self.force_new_waypoint:
            if self.is_waypoint_valid() and not self.has_reached_waypoint():
                self.node.get_logger().debug('Keeping existing waypoint')
                return self.current_waypoint

        # Reset force flag
        self.force_new_waypoint = False
        
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
        
        # Use time-bound approach instead of fixed attempts
        start_time = self.node.get_clock().now()
        max_search_time = 1.0  # Maximum time to search in seconds
        
        while True:
            # Check if we've exceeded our time limit
            current_time = self.node.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > max_search_time:
                self.node.get_logger().info('Waypoint search time limit reached')
                break

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
            
            # Calculate scores with additional stability factors
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
            
            # 4. Stability score (prefer points similar to current waypoint if it exists)
            stability_score = 0.0
            if self.current_waypoint:
                current_x = self.current_waypoint.pose.position.x
                current_y = self.current_waypoint.pose.position.y
                dist_to_current = math.sqrt((x - current_x)**2 + (y - current_y)**2)
                stability_score = 1.0 / (1.0 + dist_to_current)
            
            # Combine scores with weights
            total_score = (
                3.0 * unknown_score +    # Prioritize exploring unknown areas
                2.0 * wall_score +       # Prefer staying away from walls
                1.0 * distance_score +   # Consider distance from robot
                2.0 * stability_score    # Add stability preference
            )
            
            # Only accept new waypoint if score is significantly better
            if self.current_waypoint and total_score < self.last_best_score + self.score_improvement_threshold:
                self.waypoint_attempts += 1
                if self.waypoint_attempts < self.max_attempts:
                    continue
            
            if total_score > best_score:
                best_score = total_score
                best_point = (x, y)
        
        if best_point is None:
            return None
        
        # Once we find a valid waypoint, stick to it
        if best_point is not None:
            # Check if waypoint is near walls
            attempts = 0
            while (self.is_near_wall(best_point[0], best_point[1], map_data, 
                   resolution, origin_x, origin_y) and attempts < self.max_wall_retry):
                self.node.get_logger().warn(f'Waypoint too close to wall, finding alternative (attempt {attempts + 1})')
                best_point = self.find_alternative_waypoint(
                    best_point[0], best_point[1],
                    map_data, resolution, origin_x, origin_y
                )
                attempts += 1
            
            if attempts >= self.max_wall_retry:
                self.node.get_logger().warn('Could not find waypoint away from walls')
            
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
            
            self.current_waypoint = waypoint
            self.last_waypoint_time = self.node.get_clock().now()
            self.reached_waypoint = False
            self.node.get_logger().info('New waypoint selected')
            
        return self.current_waypoint

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
