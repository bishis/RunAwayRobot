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
        
        # Add waypoint cancellation state
        self.waypoint_cancelled = False

        # Map subscription
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        # Current map storage
        self.map_resolution = None
        self.map_origin = None
        
        # Waypoints storage
        self.waypoints = []  # This should be revalidated when map updates

        # Add minimum time between waypoint changes
        self.min_waypoint_time = 5.0  # seconds
        self.last_waypoint_change = None

        # Add last goal tracking
        self.last_goal = None
        self.min_goal_distance = 0.3  # Minimum distance between consecutive goals

    def map_callback(self, msg):
        """Process incoming map updates"""
        map_changed = False
        if self.current_map:
            # Check if map has significantly changed
            for old, new in zip(self.current_map.data, msg.data):
                if abs(old - new) > 10:  # Threshold for significant change
                    map_changed = True
                    break
        
        self.current_map = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        
        if map_changed:
            self.validate_existing_waypoints()

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
        """Check if a point is valid (not in obstacle)"""
        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((y - self.map_origin.position.y) / self.map_resolution)
        
        # Check map bounds
        if map_x < 0 or map_x >= self.current_map.info.width or \
           map_y < 0 or map_y >= self.current_map.info.height:
            return False
        
        # Check if point is in obstacle
        index = map_y * self.current_map.info.width + map_x
        return self.current_map.data[index] < 50  # < 50 means free space

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
                # Force new waypoint when current one is reached
                self.force_waypoint_change()
                # Clear current waypoint to prevent re-sending
                self.current_waypoint = None
            
            return reached
            
        except TransformException:
            return False

    def force_waypoint_change(self):
        """Force the generator to pick a new waypoint by clearing last goal"""
        self.last_goal = None
        self.force_new_waypoint = True
        self.current_waypoint = None

    def is_near_wall(self, x: float, y: float, map_data: np.ndarray, 
                    resolution: float, origin_x: float, origin_y: float) -> bool:
        """Check if a point is too close to walls with hysteresis"""
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        
        # Use different thresholds for existing vs new waypoints
        threshold = self.safety_margin
        if self.current_waypoint and \
           x == self.current_waypoint.pose.position.x and \
           y == self.current_waypoint.pose.position.y:
            threshold *= 0.8  # More permissive for existing waypoint
        
        # Check distance to nearest wall
        wall_distance = distance_transform_edt(map_data < 50) * resolution
        return wall_distance[map_y, map_x] < threshold

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

    def cancel_waypoint(self):
        """Cancel current waypoint and clear visualization"""
        self.node.get_logger().info('Cancelling current waypoint')
        self.current_waypoint = None
        self.waypoint_cancelled = True
        self.force_new_waypoint = False  # Don't force new waypoint immediately
        
        # Return empty marker array to clear visualization
        empty_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.action = Marker.DELETE
        empty_markers.markers.append(marker)
        return empty_markers

    def generate_waypoint(self):
        """Get next waypoint, either existing or new"""
        current_time = self.node.get_clock().now()
        
        # Add minimum distance between waypoints
        MIN_WAYPOINT_SEPARATION = 0.4  # Minimum 0.4 meter between waypoints
        
        # Check if minimum time has elapsed since last change
        if self.last_waypoint_change and not self.force_new_waypoint:
            time_since_change = (current_time - self.last_waypoint_change).nanoseconds / 1e9
            if time_since_change < self.min_waypoint_time:
                return self.current_waypoint
        
        # Reset waypoint cancelled flag
        self.waypoint_cancelled = False
        
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
            
            # Calculate scores with additional distance check
            dist_to_robot = math.sqrt((map_x - robot_grid_x)**2 + (map_y - robot_grid_y)**2) * resolution
            
            # Check minimum distance from previous waypoint
            too_close_to_previous = False
            if self.last_goal is not None:
                prev_x = self.last_goal.pose.position.x
                prev_y = self.last_goal.pose.position.y
                dist_to_prev = math.sqrt(
                    (x - prev_x)**2 + (y - prev_y)**2
                )
                if dist_to_prev < MIN_WAYPOINT_SEPARATION:
                    too_close_to_previous = True
                    continue  # Skip this point if too close to previous
            
            if dist_to_robot < self.min_distance or too_close_to_previous:
                continue
            
            # Score calculation with previous waypoint distance factor
            unknown_score = 1.0 / (unknown_distance[map_y, map_x] + 0.1)
            wall_score = min(wall_distance[map_y, map_x], 2.0) / 2.0
            distance_score = 1.0 - abs(dist_to_robot - self.preferred_distance) / self.preferred_distance
            
            # Add separation score
            separation_score = 1.0
            if self.last_goal is not None:
                dist_to_prev = math.sqrt(
                    (x - prev_x)**2 + (y - prev_y)**2
                )
                separation_score = min(dist_to_prev / MIN_WAYPOINT_SEPARATION, 2.0)
            
            # Updated score calculation
            total_score = (
                3.0 * unknown_score +     # Prioritize exploring unknown areas
                2.0 * wall_score +         # Prefer staying away from walls
                1.0 * distance_score +     # Consider distance from robot
                2.0 * separation_score     # Prefer points far from previous waypoint
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
            
        # Update last change time when we do change waypoint
        self.last_waypoint_change = current_time
        return self.current_waypoint

    def create_visualization_markers(self, waypoint: PoseStamped = None, is_escape: bool = False) -> MarkerArray:
        """Create visualization markers for waypoint and frontiers
        
        Args:
            waypoint: Optional waypoint to visualize
            is_escape: If True, use red color for escape waypoint
        """
        markers = MarkerArray()
        
        # Add waypoint marker if provided
        if waypoint is not None:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose = waypoint.pose
            
            # Set scale
            marker.scale.x = self.waypoint_size
            marker.scale.y = self.waypoint_size
            marker.scale.z = 0.1
            
            # Set color based on type
            if is_escape:
                # Red for escape waypoints
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                # Green for exploration waypoints
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
            
        # Rest of visualization code...
        return markers

    def validate_existing_waypoints(self):
        """Revalidate all existing waypoints against current map"""
        valid_waypoints = []
        for waypoint in self.waypoints:
            x = waypoint.pose.position.x
            y = waypoint.pose.position.y
            
            if self.is_valid_point(x, y):
                valid_waypoints.append(waypoint)
            else:
                # Try to find nearest valid point
                new_point = self.find_nearest_valid_point(x, y)
                if new_point:
                    valid_waypoints.append(new_point)
                self.node.get_logger().warn(f'Waypoint at ({x}, {y}) is now invalid, adjusted or removed')
        
        # Update waypoints list
        self.waypoints = valid_waypoints
        # Republish updated waypoints
        self.publish_waypoints()

    def find_nearest_valid_point(self, x, y, search_radius=1.0):
        """Find nearest valid point to invalid waypoint"""
        step = self.map_resolution
        max_steps = int(search_radius / step)
        
        # Search in expanding circles
        for r in range(1, max_steps):
            for theta in range(0, 360, 10):  # Check every 10 degrees
                rad = math.radians(theta)
                test_x = x + r * step * math.cos(rad)
                test_y = y + r * step * math.sin(rad)
                
                if self.is_valid_point(test_x, test_y):
                    # Create new waypoint at valid location
                    new_waypoint = PoseStamped()
                    new_waypoint.header.frame_id = 'map'
                    new_waypoint.pose.position.x = test_x
                    new_waypoint.pose.position.y = test_y
                    new_waypoint.pose.orientation = self.waypoints[0].pose.orientation  # Keep original orientation
                    return new_waypoint
        return None

    def publish_waypoints(self):
        """Publish visualization markers for current waypoints"""
        markers = MarkerArray()
        
        # Create marker for each waypoint
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position from waypoint
            marker.pose = waypoint.pose
            
            # Set scale (size)
            marker.scale.x = self.waypoint_size
            marker.scale.y = self.waypoint_size
            marker.scale.z = self.waypoint_size
            
            # Set color (blue for normal waypoints)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.6  # Semi-transparent
            
            markers.markers.append(marker)
            
            # Add line connecting waypoints
            if i > 0:
                line = Marker()
                line.header.frame_id = 'map'
                line.header.stamp = self.node.get_clock().now().to_msg()
                line.ns = 'waypoint_connections'
                line.id = i + 1000  # Offset IDs for lines
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                
                # Create line between current and previous waypoint
                prev_point = Point()
                prev_point.x = self.waypoints[i-1].pose.position.x
                prev_point.y = self.waypoints[i-1].pose.position.y
                prev_point.z = 0.0
                
                curr_point = Point()
                curr_point.x = waypoint.pose.position.x
                curr_point.y = waypoint.pose.position.y
                curr_point.z = 0.0
                
                line.points = [prev_point, curr_point]
                
                # Set line properties
                line.scale.x = 0.05  # Line width
                line.color.r = 0.0
                line.color.g = 0.5
                line.color.b = 1.0
                line.color.a = 0.4
                
                markers.markers.append(line)
        
        # Publish markers
        if hasattr(self.node, 'marker_pub'):
            self.node.marker_pub.publish(markers)


