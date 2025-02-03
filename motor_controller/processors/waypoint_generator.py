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
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import time

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

        # Add navigation feedback tracking
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Navigation state tracking
        self.navigation_start_time = None
        self.navigation_timeout = 60.0
        self.consecutive_failures = 0
        self.max_consecutive_failures = 2
        self.last_goal_status = None
        self.is_navigating = False
        
        # Subscribe to navigation status instead of state
        self.nav_status_sub = self.node.create_subscription(
            GoalStatus,
            '/navigation/status',  # Updated topic name
            self.navigation_status_callback,
            10
        )

        # Add tracking of previous waypoints
        self.previous_waypoints = []
        self.max_previous_waypoints = 10
        self.failed_waypoints = set()
        self.last_successful_waypoint = None
        self.min_waypoint_distance = 1.0  # Minimum distance from previous waypoints

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
            
            return reached
            
        except TransformException:
            return False

    def force_waypoint_change(self):
        """Force the generator to pick a new waypoint"""
        self.force_new_waypoint = True
        self.current_waypoint = None
        self.navigation_start_time = None
        self.is_navigating = False

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

    def generate_waypoint(self) -> PoseStamped:
        """Generate a single new waypoint prioritizing unexplored areas"""
        # Check for navigation issues
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.node.get_logger().warn('Too many consecutive failures, waiting before generating new waypoint')
            time.sleep(5.0)  # Wait before trying again
            self.consecutive_failures = 0
            
        # Check for timeout
        self.check_navigation_timeout()
            
        # Don't generate new waypoint if cancelled until explicitly requested
        if self.waypoint_cancelled and not self.force_new_waypoint:
            return None
            
        # Reset cancelled state if forcing new waypoint
        if self.force_new_waypoint:
            self.waypoint_cancelled = False
            self.navigation_start_time = None
            self.is_navigating = False
            
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
        
        # Calculate scores
        exploration_score = 1.0 / (unknown_distance + 0.1)  # High score near unknown areas
        safety_score = np.minimum(wall_distance / self.safety_margin, 1.0)  # High score away from walls
        
        # Combine scores
        total_score = exploration_score * safety_score
        
        # Zero out scores for invalid areas
        total_score[walls] = 0
        total_score[unknown_area] = 0
        
        # Zero out areas near previous waypoints
        for prev_wp in self.previous_waypoints:
            px = int((prev_wp.pose.position.x - origin_x) / resolution)
            py = int((prev_wp.pose.position.y - origin_y) / resolution)
            radius = int(self.min_waypoint_distance / resolution)
            y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
            mask = x_indices*x_indices + y_indices*y_indices <= radius*radius
            
            # Ensure indices are within bounds
            y_start = max(0, py-radius)
            y_end = min(total_score.shape[0], py+radius+1)
            x_start = max(0, px-radius)
            x_end = min(total_score.shape[1], px+radius+1)
            
            # Apply mask
            mask_height = y_end - y_start
            mask_width = x_end - x_start
            total_score[y_start:y_end, x_start:x_end][mask[:mask_height, :mask_width]] = 0
        
        # Get candidate points (top 10 scores)
        candidates = []
        for _ in range(10):
            if np.max(total_score) > 0:
                y, x = np.unravel_index(np.argmax(total_score), total_score.shape)
                world_x = x * resolution + origin_x
                world_y = y * resolution + origin_y
                score = total_score[y, x]
                candidates.append((world_x, world_y, score))
                # Clear area around this candidate
                radius = int(self.min_waypoint_distance / resolution)
                y_indices, x_indices = np.ogrid[-radius:radius+1, -radius:radius+1]
                mask = x_indices*x_indices + y_indices*y_indices <= radius*radius
                y_start = max(0, y-radius)
                y_end = min(total_score.shape[0], y+radius+1)
                x_start = max(0, x-radius)
                x_end = min(total_score.shape[1], x+radius+1)
                mask_height = y_end - y_start
                mask_width = x_end - x_start
                total_score[y_start:y_end, x_start:x_end][mask[:mask_height, :mask_width]] = 0
        
        # Randomly select from top candidates with probability proportional to score
        if candidates:
            scores = np.array([c[2] for c in candidates])
            probs = scores / np.sum(scores)
            chosen_idx = np.random.choice(len(candidates), p=probs)
            x, y, _ = candidates[chosen_idx]
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.node.get_clock().now().to_msg()
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            
            # Set orientation towards unexplored area
            theta = math.atan2(y, x)  # Simple orientation towards point
            waypoint.pose.orientation.z = math.sin(theta/2)
            waypoint.pose.orientation.w = math.cos(theta/2)
            
            # Update previous waypoints list
            self.previous_waypoints.append(waypoint)
            if len(self.previous_waypoints) > self.max_previous_waypoints:
                self.previous_waypoints.pop(0)
            
            self.current_waypoint = waypoint
            return waypoint
        
        return None

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
        # Implement the logic to republish updated waypoints
        pass

    def navigation_status_callback(self, msg):
        """Handle navigation status updates"""
        if msg.status == GoalStatus.ABORTED:  # Failed
            if self.current_waypoint:
                # Add failed waypoint to set
                self.failed_waypoints.add(
                    (self.current_waypoint.pose.position.x,
                     self.current_waypoint.pose.position.y)
                )
            self.node.get_logger().warn('Navigation failed, forcing new waypoint')
            self.consecutive_failures += 1
            self.force_waypoint_change()
        elif msg.status == GoalStatus.SUCCEEDED:
            self.consecutive_failures = 0
            if self.current_waypoint:
                self.last_successful_waypoint = self.current_waypoint
            self.force_waypoint_change()
        elif msg.status == GoalStatus.ACCEPTED:  # Planning
            if not self.navigation_start_time:
                self.navigation_start_time = time.time()

    def check_navigation_timeout(self):
        """Check if current navigation has timed out"""
        if self.navigation_start_time and self.is_navigating:
            elapsed_time = time.time() - self.navigation_start_time
            if elapsed_time > self.navigation_timeout:
                self.node.get_logger().warn('Navigation timeout, forcing new waypoint')
                self.force_waypoint_change()
                return True
        return False

    def send_goal(self, waypoint: PoseStamped):
        """Send goal to navigation stack and track its status"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('Navigation action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.navigation_start_time = time.time()
        self.is_navigating = True
        
        # Send the goal
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Check estimated time remaining
        if hasattr(feedback, 'estimated_time_remaining'):
            if feedback.estimated_time_remaining.sec > self.navigation_timeout:
                self.node.get_logger().warn('Estimated time too long, considering new waypoint')
                self.force_waypoint_change()

    def update_map(self, map_msg: OccupancyGrid):
        """Update stored map and validate current waypoint"""
        # Process map update similar to map_callback
        map_changed = False
        if self.current_map:
            # Check if map has significantly changed
            for old, new in zip(self.current_map.data, map_msg.data):
                if abs(old - new) > 10:  # Threshold for significant change
                    map_changed = True
                    break
        
        self.current_map = map_msg
        self.map_resolution = map_msg.info.resolution
        self.map_origin = map_msg.info.origin
        
        # Check if current waypoint is still valid
        if self.current_waypoint and self.current_map:
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            x = self.current_waypoint.pose.position.x
            y = self.current_waypoint.pose.position.y
            
            # Check if point is still in free space
            if not self.is_valid_point(x, y):
                self.node.get_logger().warn('Current waypoint is no longer in free space, forcing new waypoint')
                self.force_waypoint_change()
                return
            
            # Check if too close to walls
            if self.is_near_wall(x, y, map_data, 
                               self.current_map.info.resolution,
                               self.current_map.info.origin.position.x,
                               self.current_map.info.origin.position.y):
                self.node.get_logger().warn('Current waypoint is too close to wall, forcing new waypoint')
                self.force_waypoint_change()
                return
        
        # If map changed significantly, validate all waypoints
        if map_changed:
            self.validate_existing_waypoints()
