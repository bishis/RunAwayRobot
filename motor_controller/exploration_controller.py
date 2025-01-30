#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
from sklearn.cluster import DBSCAN

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        # Parameters
        self.declare_parameter('min_frontier_size', 5)    # Minimum cells for a valid frontier
        self.declare_parameter('exploration_radius', 1.0) # How far to move for each goal
        self.declare_parameter('wall_threshold', 80)      # Value to consider as wall (0-100)
        self.declare_parameter('unknown_threshold', -1)   # Value for unknown cells
        self.declare_parameter('replan_distance', 0.5)  # Distance to trigger replanning
        self.declare_parameter('coverage_threshold', 0.85)  # When room is considered "mapped enough"
        self.declare_parameter('grid_size', 0.5)  # Size of grid cells for coverage pattern
        self.declare_parameter('goal_timeout', 30.0)  # Seconds before giving up on a goal
        self.declare_parameter('max_retries', 3)      # Max attempts for a single goal
        
        # Get parameters
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.wall_threshold = self.get_parameter('wall_threshold').value
        self.unknown_threshold = self.get_parameter('unknown_threshold').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.grid_size = self.get_parameter('grid_size').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        
        # Set up Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration_markers', 10)
        
        # Create timer for exploration control
        self.create_timer(1.0, self.exploration_loop)
        
        # Exploration state
        self.current_map = None
        self.is_navigating = False
        self.current_goal = None
        self.marker_id = 0
        self.last_map = None
        self.exploration_state = 'EXPLORING'  # States: 'EXPLORING', 'COVERAGE'
        
        # Add tracking variables
        self.goal_start_time = None
        self.current_retries = 0
        self.failed_goals = set()  # Track failed goal positions
        
        self.get_logger().info('Exploration controller initialized')

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map data and check if we need to replan"""
        if self.is_navigating and self.current_goal:
            # Check if path to current goal is blocked
            if self.is_path_blocked(msg):
                self.get_logger().info('Path to goal blocked, replanning...')
                self.cancel_current_goal()
                self.current_goal = None
        
        self.current_map = msg

    def find_frontiers(self):
        """Find and cluster frontier regions"""
        if not self.current_map:
            return []

        # Get map dimensions and info
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        map_data = np.array(self.current_map.data).reshape(height, width)
        
        # Create binary maps with more lenient thresholds
        unknown = map_data == -1  # Changed from self.unknown_threshold
        walls = map_data >= 80    # Increased wall threshold
        free = (map_data >= 0) & (map_data < 50)  # More lenient free space definition
        
        # Find frontiers
        frontiers = []
        kernel = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
        
        # Reduced margin to find more frontiers
        margin = 2  # Reduced from 5
        for y in range(margin, height - margin):
            for x in range(margin, width - margin):
                if not free[y, x]:
                    continue
                
                # Check if cell is next to unknown area
                window = unknown[y-1:y+2, x-1:x+2]
                if np.any(window * kernel):
                    # Convert to world coordinates
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    
                    # More lenient bounds checking
                    if (origin_x < world_x < origin_x + width * resolution and
                        origin_y < world_y < origin_y + height * resolution):
                        
                        # Allow more walls nearby
                        wall_check = walls[y-1:y+2, x-1:x+2]
                        if np.sum(wall_check) <= 4:  # Increased from 3
                            frontiers.append((x, y))
        
        if not frontiers:
            return []

        # Cluster frontiers with more lenient parameters
        frontier_points = np.array(frontiers)
        
        clustering = DBSCAN(
            eps=8,           # Increased from 6
            min_samples=1    # Reduced from 2
        ).fit(frontier_points)
        
        # Group points by cluster
        clusters = {}
        for i, label in enumerate(clustering.labels_):
            if label == -1:  # Include single points
                clusters[len(clusters)] = [frontier_points[i]]
            else:
                if label not in clusters:
                    clusters[label] = []
                clusters[label].append(frontier_points[i])
        
        grouped_frontiers = [np.array(points) for points in clusters.values()]
        
        # More lenient minimum size
        min_frontier_size = 1  # Reduced from 2
        grouped_frontiers = [f for f in grouped_frontiers if len(f) >= min_frontier_size]
        
        return grouped_frontiers

    def publish_visualization_markers(self, frontiers, selected_goal=None):
        """Publish visualization markers for debugging"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = 'exploration'
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Show frontiers
        for frontier in frontiers:
            frontier_marker = Marker()
            frontier_marker.header.frame_id = 'map'
            frontier_marker.header.stamp = self.get_clock().now().to_msg()
            frontier_marker.ns = 'exploration'
            frontier_marker.id = self.marker_id
            self.marker_id += 1
            frontier_marker.type = Marker.POINTS
            frontier_marker.action = Marker.ADD
            frontier_marker.scale.x = 0.05
            frontier_marker.scale.y = 0.05
            frontier_marker.color.r = 0.0
            frontier_marker.color.g = 1.0
            frontier_marker.color.b = 0.0
            frontier_marker.color.a = 1.0
            
            # Convert grid cells to world coordinates
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            for x, y in frontier:
                point = Point()
                point.x = origin_x + x * resolution
                point.y = origin_y + y * resolution
                point.z = 0.1
                frontier_marker.points.append(point)
            
            marker_array.markers.append(frontier_marker)
        
        # Show selected goal
        if selected_goal:
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'exploration'
            goal_marker.id = self.marker_id
            self.marker_id += 1
            goal_marker.type = Marker.ARROW
            goal_marker.action = Marker.ADD
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.1
            goal_marker.scale.z = 0.1
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            goal_marker.pose = selected_goal.pose
            
            marker_array.markers.append(goal_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)

    def find_exploration_goal(self):
        """Find the next exploration goal with better validation"""
        if not self.current_map:
            return None
        
        try:
            # Get current robot pose
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            frontier_clusters = self.find_frontiers()
            if not frontier_clusters:
                return None
            
            # Get map data for validation
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            # Score and validate frontiers
            valid_clusters = []
            scores = []
            
            for cluster in frontier_clusters:
                center = np.mean(cluster, axis=0)
                
                # Convert to world coordinates
                world_x = origin_x + center[0] * resolution
                world_y = origin_y + center[1] * resolution
                
                # Create temporary goal to check validity
                temp_goal = PoseStamped()
                temp_goal.header.frame_id = 'map'
                temp_goal.pose.position.x = world_x
                temp_goal.pose.position.y = world_y
                
                # Only consider if goal is valid
                if self.is_valid_goal(temp_goal):
                    valid_clusters.append(cluster)
                    
                    # Calculate score
                    dist = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
                    score = len(cluster) / (dist + 1.0)  # Favor larger clusters that are closer
                    scores.append(score)
            
            if not valid_clusters:
                return None
            
            # Select best valid cluster
            best_cluster = valid_clusters[np.argmax(scores)]
            center = np.mean(best_cluster, axis=0)
            
            # Create goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = origin_x + center[0] * resolution
            goal.pose.position.y = origin_y + center[1] * resolution
            goal.pose.position.z = 0.0
            
            # Set orientation towards center
            angle = math.atan2(
                center[1] - robot_y,
                center[0] - robot_x
            )
            goal.pose.orientation.w = math.cos(angle / 2)
            goal.pose.orientation.z = math.sin(angle / 2)
            
            # Double-check final goal validity
            if not self.is_valid_goal(goal):
                self.get_logger().warn('Final goal validation failed')
                return None
            
            self.get_logger().info(
                f'Selected goal: grid=({center[0]:.1f}, {center[1]:.1f}), '
                f'world=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
            )
            
            return goal
            
        except Exception as e:
            self.get_logger().error(f'Error finding exploration goal: {e}')
            return None

    def is_valid_goal(self, goal_pose):
        """Check if goal position is valid"""
        if not self.current_map:
            return False
        
        try:
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            # Convert goal to grid coordinates
            grid_x = int((goal_pose.pose.position.x - origin_x) / resolution)
            grid_y = int((goal_pose.pose.position.y - origin_y) / resolution)
            
            # Basic bounds check
            if (grid_x < 0 or grid_x >= self.current_map.info.width or
                grid_y < 0 or grid_y >= self.current_map.info.height):
                return False
            
            # Get map data
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            
            # Check immediate surroundings (smaller area)
            safety_radius = 2  # Reduced from 5
            for dy in range(-safety_radius, safety_radius + 1):
                for dx in range(-safety_radius, safety_radius + 1):
                    check_x = grid_x + dx
                    check_y = grid_y + dy
                    if (0 <= check_x < self.current_map.info.width and 
                        0 <= check_y < self.current_map.info.height):
                        cell_value = map_data[check_y, check_x]
                        if cell_value >= 80:  # Increased threshold
                            return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error checking goal validity: {e}')
            return False

    def check_goal_timeout(self):
        """Check if current goal has timed out"""
        if not self.goal_start_time:
            return False
        
        time_elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        return time_elapsed > self.goal_timeout

    def send_goal(self, goal_pose):
        """Send navigation goal to Nav2 with validation"""
        if not self.is_valid_goal(goal_pose):
            self.get_logger().warn('Invalid goal, finding new goal')
            return False
        
        self.get_logger().info('Sending new navigation goal')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.nav_client.wait_for_server()
        
        self.current_goal = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.current_goal.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        self.goal_start_time = self.get_clock().now()
        return True

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation goal result"""
        result = future.result().result
        self.is_navigating = False
        self.get_logger().info('Navigation completed')

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Can add feedback processing here if needed

    def is_path_blocked(self, new_map):
        """Check if the path to current goal is blocked by new obstacles"""
        if not self.last_map or not self.current_goal:
            self.last_map = new_map
            return False
        
        try:
            # Get current robot pose
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convert positions to grid coordinates
            resolution = new_map.info.resolution
            origin_x = new_map.info.origin.position.x
            origin_y = new_map.info.origin.position.y
            
            robot_grid_x = int((robot_x - origin_x) / resolution)
            robot_grid_y = int((robot_y - origin_y) / resolution)
            
            goal_grid_x = int((self.current_goal.pose.position.x - origin_x) / resolution)
            goal_grid_y = int((self.current_goal.pose.position.y - origin_y) / resolution)
            
            # Check for new obstacles along rough path to goal
            # Using Bresenham's line algorithm to check cells between robot and goal
            cells = self.get_line_cells(robot_grid_x, robot_grid_y, goal_grid_x, goal_grid_y)
            
            new_map_data = np.array(new_map.data).reshape(new_map.info.height, new_map.info.width)
            old_map_data = np.array(self.last_map.data).reshape(self.last_map.info.height, self.last_map.info.width)
            
            for x, y in cells:
                if (0 <= x < new_map.info.width and 0 <= y < new_map.info.height):
                    # Check if cell changed from unknown/free to occupied
                    if (old_map_data[y, x] <= self.wall_threshold and 
                        new_map_data[y, x] > self.wall_threshold):
                        self.get_logger().info(f'New obstacle detected at ({x}, {y})')
                        return True
            
            self.last_map = new_map
            return False
        
        except Exception as e:
            self.get_logger().warning(f'Error checking path: {e}')
            return False

    def get_line_cells(self, x0, y0, x1, y1):
        """Get cells along a line using Bresenham's algorithm"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            
        cells.append((x, y))
        return cells

    def cancel_current_goal(self):
        """Cancel the current navigation goal"""
        if self.current_goal and self.is_navigating:
            # Cancel the goal using the correct method
            future = self.nav_client.cancel_goal()
            # Wait for cancellation to complete
            rclpy.spin_until_future_complete(self, future)
            self.is_navigating = False
            self.get_logger().info('Cancelled current navigation goal')

    def check_room_coverage(self):
        """Check if enough of the room has been mapped"""
        if not self.current_map:
            return False
        
        # Get map data
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Find the actual room boundaries (known space)
        known = map_data != -1
        if not np.any(known):
            return False
        
        # Get the bounding box of known space
        known_y, known_x = np.where(known)
        min_x, max_x = np.min(known_x), np.max(known_x)
        min_y, max_y = np.min(known_y), np.max(known_y)
        
        # Get the actual room area (known space within bounds)
        room_area = map_data[min_y:max_y+1, min_x:max_x+1]
        
        # Count cells
        total_cells = (max_x - min_x + 1) * (max_y - min_y + 1)
        unknown_cells = np.sum(room_area == -1)
        
        # Calculate coverage of actual room
        coverage = 1.0 - (unknown_cells / total_cells)
        
        self.get_logger().info(f'Room coverage: {coverage:.2%}')
        
        if coverage >= self.coverage_threshold:
            self.get_logger().info(
                f'Room coverage threshold reached ({coverage:.2%} >= {self.coverage_threshold:.2%}), '
                'switching to coverage pattern'
            )
            return True
        return False

    def generate_coverage_pattern(self):
        """Generate a grid pattern to cover the mapped area"""
        if not self.current_map:
            return None
        
        try:
            # Get current robot pose
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Get map data and info
            map_data = np.array(self.current_map.data).reshape(
                self.current_map.info.height,
                self.current_map.info.width
            )
            resolution = self.current_map.info.resolution
            origin_x = self.current_map.info.origin.position.x
            origin_y = self.current_map.info.origin.position.y
            
            # Find boundaries of known space
            known_y, known_x = np.where(map_data != -1)
            if len(known_x) == 0 or len(known_y) == 0:
                return None
            
            min_x = origin_x + min(known_x) * resolution
            max_x = origin_x + max(known_x) * resolution
            min_y = origin_y + min(known_y) * resolution
            max_y = origin_y + max(known_y) * resolution
            
            # Generate grid points
            x_points = np.arange(min_x, max_x, self.grid_size)
            y_points = np.arange(min_y, max_y, self.grid_size)
            
            # Create snake pattern
            goals = []
            for i, y in enumerate(y_points):
                row_x = x_points if i % 2 == 0 else x_points[::-1]
                for x in row_x:
                    # Convert to grid coordinates
                    grid_x = int((x - origin_x) / resolution)
                    grid_y = int((y - origin_y) / resolution)
                    
                    # Check if point is in free space
                    if (0 <= grid_x < self.current_map.info.width and 
                        0 <= grid_y < self.current_map.info.height and
                        map_data[grid_y, grid_x] == 0):  # Free space
                        
                        goal = PoseStamped()
                        goal.header.frame_id = 'map'
                        goal.header.stamp = self.get_clock().now().to_msg()
                        goal.pose.position.x = x
                        goal.pose.position.y = y
                        goal.pose.position.z = 0.0
                        goal.pose.orientation.w = 1.0
                        goals.append(goal)
            
            return goals
            
        except Exception as e:
            self.get_logger().error(f'Error generating coverage pattern: {e}')
            return None

    def exploration_loop(self):
        """Main exploration control loop with timeout handling"""
        if self.is_navigating:
            # Check for timeout
            if self.check_goal_timeout():
                self.get_logger().warn('Goal timeout reached')
                self.cancel_current_goal()
                
                # Track failed goal
                if self.current_goal:
                    resolution = self.current_map.info.resolution
                    origin_x = self.current_map.info.origin.position.x
                    origin_y = self.current_map.info.origin.position.y
                    grid_x = int((self.current_goal.pose.position.x - origin_x) / resolution)
                    grid_y = int((self.current_goal.pose.position.y - origin_y) / resolution)
                    self.failed_goals.add((grid_x, grid_y))
                
                self.current_retries += 1
                if self.current_retries >= self.max_retries:
                    self.get_logger().warn('Max retries reached, switching goals')
                    self.current_retries = 0
                    self.is_navigating = False
            return

        # Reset retry counter for new goals
        self.current_retries = 0
        
        # Check if we should switch to coverage pattern
        if self.exploration_state == 'EXPLORING' and self.check_room_coverage():
            self.exploration_state = 'COVERAGE'
            self.coverage_goals = self.generate_coverage_pattern()
            self.current_coverage_goal = 0
            self.failed_goals.clear()  # Reset failed goals for new pattern
        
        # Handle different states
        if self.exploration_state == 'EXPLORING':
            goal_pose = self.find_exploration_goal()
            if goal_pose and self.send_goal(goal_pose):
                self.current_goal = goal_pose
            else:
                self.get_logger().warn('No valid exploration goal found')
            
        elif self.exploration_state == 'COVERAGE':
            while (self.coverage_goals and 
                   self.current_coverage_goal < len(self.coverage_goals)):
                goal = self.coverage_goals[self.current_coverage_goal]
                if self.send_goal(goal):
                    self.current_goal = goal
                    break
                self.current_coverage_goal += 1
            else:
                self.get_logger().info('Coverage pattern completed or no valid goals remain')

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