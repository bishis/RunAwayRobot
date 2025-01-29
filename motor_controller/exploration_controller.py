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
        
        # Get parameters
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.wall_threshold = self.get_parameter('wall_threshold').value
        self.unknown_threshold = self.get_parameter('unknown_threshold').value
        
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
        
        self.get_logger().info('Exploration controller initialized')

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map data and check if we need to replan"""
        if self.is_navigating and self.current_goal:
            # Check if path to current goal is blocked
            if self.is_path_blocked(msg):
                self.get_logger().info('Path to goal blocked, replanning...')
                self.cancel_current_goal()
                self.find_new_goal()
        
        self.current_map = msg

    def find_frontiers(self):
        """Find and cluster frontier regions"""
        if not self.current_map:
            return []

        # Convert map to numpy array
        width = self.current_map.info.width
        height = self.current_map.info.height
        map_data = np.array(self.current_map.data).reshape(height, width)
        
        # Create binary maps
        unknown = map_data == self.unknown_threshold
        free = map_data < self.wall_threshold
        free[unknown] = False
        
        # Find frontiers (free cells next to unknown cells)
        kernel = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])
        frontiers = []
        
        # Iterate through free cells
        for y in range(1, height-1):
            for x in range(1, width-1):
                if not free[y, x]:
                    continue
                    
                # Check if cell is next to unknown area
                window = unknown[y-1:y+2, x-1:x+2]
                if np.any(window * kernel):
                    frontiers.append((x, y))
        
        # Cluster frontiers using DBSCAN
        if not frontiers:
            return []
        
        # Convert to numpy array for clustering
        frontier_points = np.array(frontiers)
        
        # Use DBSCAN to cluster points
        clustering = DBSCAN(
            eps=5,           # Maximum distance between points in same cluster
            min_samples=3    # Minimum points to form a cluster
        ).fit(frontier_points)
        
        # Group points by cluster
        clusters = {}
        for i, label in enumerate(clustering.labels_):
            if label == -1:  # Noise points
                continue
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(frontier_points[i])
        
        # Convert clusters to list of point groups
        grouped_frontiers = [np.array(points) for points in clusters.values()]
        
        # Filter small clusters
        grouped_frontiers = [f for f in grouped_frontiers if len(f) >= self.min_frontier_size]
        
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
        """Find the next exploration goal based on clustered frontiers"""
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
            robot_pos = np.array([robot_x, robot_y])
        except Exception as e:
            self.get_logger().warning(f'Could not get robot pose: {e}')
            return None
        
        # Find clustered frontiers
        frontier_clusters = self.find_frontiers()
        if not frontier_clusters:
            return None
        
        # Convert robot position to grid coordinates
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        map_width = self.current_map.info.width
        map_height = self.current_map.info.height
        
        robot_grid_x = int((robot_x - origin_x) / resolution)
        robot_grid_y = int((robot_y - origin_y) / resolution)
        
        # Get map data as numpy array
        map_data = np.array(self.current_map.data).reshape(map_height, map_width)
        
        # Find best cluster based on size, distance, and accessibility
        best_cluster = None
        best_score = float('-inf')
        
        for cluster in frontier_clusters:
            # Calculate cluster center
            center = np.mean(cluster, axis=0)
            
            # Skip clusters outside map bounds
            if (center[0] < 2 or center[0] >= map_width - 2 or 
                center[1] < 2 or center[1] >= map_height - 2):
                continue
            
            # Check if area around cluster center is accessible
            center_x = int(center[0])
            center_y = int(center[1])
            area = map_data[center_y-2:center_y+3, center_x-2:center_x+3]
            if np.any(area > self.wall_threshold):  # Skip if there are walls nearby
                continue
            
            # Calculate distance to cluster
            dist = np.sqrt((center[0] - robot_grid_x)**2 + (center[1] - robot_grid_y)**2)
            
            # Skip if too close (might be unreachable due to local obstacles)
            if dist < 5:  # Skip points less than 5 cells away
                continue
            
            # Score based on size and distance (prefer larger clusters at moderate distances)
            cluster_size = len(cluster)
            # Gaussian-like distance weighting - prefer points not too close and not too far
            distance_score = np.exp(-((dist - 20) ** 2) / (2 * 15 ** 2))
            score = cluster_size * distance_score
            
            if score > best_score:
                best_score = score
                best_cluster = cluster
        
        if best_cluster is None:
            # If no good clusters found, try to recover by moving to a more open area
            self.get_logger().warn('No valid frontiers found, looking for recovery point')
            return self.find_recovery_goal()
        
        # Create goal at cluster center
        center = np.mean(best_cluster, axis=0)
        
        # Ensure goal is within map bounds and in free space
        center[0] = np.clip(center[0], 2, map_width - 3)
        center[1] = np.clip(center[1], 2, map_height - 3)
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Convert back to world coordinates
        goal.pose.position.x = origin_x + center[0] * resolution
        goal.pose.position.y = origin_y + center[1] * resolution
        goal.pose.position.z = 0.0
        
        # Set orientation towards center of frontier
        angle = math.atan2(
            center[1] - robot_grid_y,
            center[0] - robot_grid_x
        )
        goal.pose.orientation.w = math.cos(angle / 2)
        goal.pose.orientation.z = math.sin(angle / 2)
        
        self.get_logger().info(
            f'Selected goal: grid=({center[0]:.1f}, {center[1]:.1f}), '
            f'world=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )
        
        return goal

    def find_recovery_goal(self):
        """Find a recovery goal in an open area when stuck"""
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
        except Exception as e:
            return None
        
        # Convert to grid coordinates
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        robot_grid_x = int((robot_x - origin_x) / resolution)
        robot_grid_y = int((robot_y - origin_y) / resolution)
        
        # Get map data
        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )
        
        # Look for open areas in increasing radius
        for radius in range(5, 20, 2):
            y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
            mask = x**2 + y**2 <= radius**2
            
            for angle in np.linspace(0, 2*np.pi, 16):
                test_x = robot_grid_x + int(radius * np.cos(angle))
                test_y = robot_grid_y + int(radius * np.sin(angle))
                
                # Check if point is within map bounds
                if (test_x < 2 or test_x >= self.current_map.info.width - 2 or
                    test_y < 2 or test_y >= self.current_map.info.height - 2):
                    continue
                
                # Check if area is clear
                area = map_data[test_y-2:test_y+3, test_x-2:test_x+3]
                if np.all(area < self.wall_threshold):
                    # Create recovery goal
                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x = origin_x + test_x * resolution
                    goal.pose.position.y = origin_y + test_y * resolution
                    goal.pose.position.z = 0.0
                    
                    # Orient towards open space
                    goal.pose.orientation.w = math.cos(angle / 2)
                    goal.pose.orientation.z = math.sin(angle / 2)
                    
                    self.get_logger().info('Found recovery goal in open space')
                    return goal
        
        return None

    def send_goal(self, goal_pose):
        """Send navigation goal to Nav2"""
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
            # Cancel the goal
            self.nav_client.cancel_goal_async()
            self.is_navigating = False
            self.get_logger().info('Cancelled current navigation goal')

    def find_new_goal(self):
        """Find a new goal when current path is blocked"""
        # Get current robot pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_pos = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])
            
            # Find frontiers excluding the blocked area
            frontiers = self.find_frontiers()
            if not frontiers:
                return
            
            # Score frontiers based on distance and accessibility
            best_frontier = None
            best_score = float('-inf')
            
            for frontier in frontiers:
                center = np.mean(frontier, axis=0)
                
                # Skip if too close to current blocked path
                if self.is_near_blocked_path(center):
                    continue
                
                # Score based on distance and size
                dist = np.linalg.norm(center - robot_pos)
                size = len(frontier)
                score = size / (dist + 1)
                
                if score > best_score:
                    best_score = score
                    best_frontier = frontier
            
            if best_frontier is not None:
                goal = self.create_goal_from_frontier(best_frontier)
                self.current_goal = goal
                self.send_goal(goal)
                
        except Exception as e:
            self.get_logger().error(f'Error finding new goal: {e}')

    def is_near_blocked_path(self, point, threshold=1.0):
        """Check if point is near the previously blocked path"""
        if not hasattr(self, 'blocked_areas'):
            self.blocked_areas = []
            return False
        
        for blocked_point in self.blocked_areas:
            dist = np.linalg.norm(point - blocked_point)
            if dist < threshold:
                return True
        return False

    def exploration_loop(self):
        """Main exploration control loop"""
        if self.is_navigating:
            return
            
        # Find and send new exploration goal
        goal_pose = self.find_exploration_goal()
        if goal_pose:
            self.current_goal = goal_pose
            self.send_goal(goal_pose)
        else:
            self.get_logger().warn('No valid exploration goal found')

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