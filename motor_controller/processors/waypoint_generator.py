import math
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class WaypointGenerator(Node):
    """ROS2 node for generating and following waypoints for autonomous exploration."""

    def __init__(self, robot_radius=0.2, safety_margin=0.3, num_waypoints=5):
        super().__init__('waypoint_generator')
        
        # Navigation parameters
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.num_waypoints = num_waypoints
        self.min_waypoint_spacing = robot_radius * 3
        
        # Initialize the waypoint following client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for navigation server
        self.get_logger().info('Waiting for navigation action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Navigation server connected!')
        
    def generate_waypoints(self, current_pose, map_data, map_info, is_valid_point, map_to_world):
        """Generate waypoints prioritizing unexplored areas and boundary mapping."""
        if not current_pose or not map_info:
            return []
        
        points = []
        x = current_pose.position.x
        y = current_pose.position.y
        
        frontiers = self._find_unexplored_frontier(map_data, map_info, map_to_world)
        boundaries = self._find_boundary_points(map_data, map_info, map_to_world)
        
        candidate_points = self._get_candidate_points(frontiers, boundaries, points, x, y)
        return self._select_waypoints(candidate_points, points, is_valid_point)

    def _is_point_too_close(self, new_x, new_y, existing_points):
        for point in existing_points:
            dx = new_x - point.x
            dy = new_y - point.y
            if math.sqrt(dx*dx + dy*dy) < self.min_waypoint_spacing:
                return True
        return False

    def _find_unexplored_frontier(self, map_data, map_info, map_to_world):
        frontiers = []
        height, width = map_data.shape
        
        # Improved frontier detection with clustering
        visited = np.zeros((height, width), dtype=bool)
        min_frontier_size = 3  # Minimum frontier cluster size
        
        for my in range(1, height-1):
            for mx in range(1, width-1):
                if map_data[my, mx] == 0 and not visited[my, mx]:  # Free space
                    frontier_cluster = []
                    stack = [(mx, my)]
                    
                    while stack:
                        cx, cy = stack.pop()
                        if visited[cy, cx]:
                            continue
                            
                        visited[cy, cx] = True
                        has_unknown = False
                        
                        # Check 8-connected neighbors
                        for dy in [-1, 0, 1]:
                            for dx in [-1, 0, 1]:
                                nx, ny = cx + dx, cy + dy
                                if (0 <= ny < height and 0 <= nx < width):
                                    if map_data[ny, nx] == -1:  # Unknown space
                                        has_unknown = True
                                    elif (map_data[ny, nx] == 0 and 
                                          not visited[ny, nx]):
                                        stack.append((nx, ny))
                        
                        if has_unknown:
                            frontier_cluster.append((cx, cy))
                    
                    # Only add large enough frontier clusters
                    if len(frontier_cluster) >= min_frontier_size:
                        # Add center point of the cluster
                        center_x = sum(x for x, _ in frontier_cluster) / len(frontier_cluster)
                        center_y = sum(y for _, y in frontier_cluster) / len(frontier_cluster)
                        wx, wy = map_to_world(int(center_x), int(center_y))
                        if wx is not None and wy is not None:
                            frontiers.append((wx, wy))
        
        return frontiers

    def _find_boundary_points(self, map_data, map_info, map_to_world):
        boundaries = []
        height, width = map_data.shape
        min_clearance = 3  # Minimum cells from obstacles
        max_clearance = 8  # Maximum cells from obstacles
        
        # Create distance transform from obstacles
        obstacle_map = (map_data > 50).astype(np.uint8)
        distance_map = np.zeros((height, width), dtype=np.float32)
        
        # Calculate distance to nearest obstacle for each cell
        for my in range(height):
            for mx in range(width):
                if map_data[my, mx] == 0:  # Free space
                    min_dist = float('inf')
                    for dy in range(-max_clearance, max_clearance + 1):
                        ny = my + dy
                        if 0 <= ny < height:
                            for dx in range(-max_clearance, max_clearance + 1):
                                nx = mx + dx
                                if 0 <= nx < width and obstacle_map[ny, nx]:
                                    dist = math.sqrt(dx*dx + dy*dy)
                                    min_dist = min(min_dist, dist)
                    distance_map[my, mx] = min_dist
        
        # Find boundary points with good clearance
        for my in range(1, height-1):
            for mx in range(1, width-1):
                if (map_data[my, mx] == 0 and 
                    min_clearance <= distance_map[my, mx] <= max_clearance):
                    # Check if point is on the boundary between open and explored space
                    has_unknown = False
                    for dy in [-2, -1, 0, 1, 2]:
                        for dx in [-2, -1, 0, 1, 2]:
                            if (0 <= my+dy < height and 0 <= mx+dx < width):
                                if map_data[my+dy, mx+dx] == -1:
                                    has_unknown = True
                                    break
                        if has_unknown:
                            break
                    
                    if has_unknown:
                        wx, wy = map_to_world(mx, my)
                        if wx is not None and wy is not None:
                            boundaries.append((wx, wy))
        
        return boundaries

    def _get_candidate_points(self, frontiers, boundaries, points, x, y):
        candidates = []
        
        for wx, wy in frontiers:
            if not self._is_point_too_close(wx, wy, points):
                candidates.append(('frontier', wx, wy))
        
        for wx, wy in boundaries:
            if not self._is_point_too_close(wx, wy, points):
                candidates.append(('boundary', wx, wy))
        
        candidates.sort(key=lambda p: math.sqrt((p[1]-x)**2 + (p[2]-y)**2))
        return candidates

    def _select_waypoints(self, candidates, points, is_valid_point):
        while len(points) < self.num_waypoints and candidates:
            target_type = 'frontier' if len(points) % 2 == 0 else 'boundary'
            
            for i, (point_type, wx, wy) in enumerate(candidates):
                if point_type == target_type:
                    if is_valid_point(wx, wy) and not self._is_point_too_close(wx, wy, points):
                        point = Point(x=wx, y=wy, z=0.0)
                        points.append(point)
                        candidates.pop(i)
                        break
            else:
                target_type = 'boundary' if target_type == 'frontier' else 'frontier'
                continue
        
        return points

    def _has_unknown_neighbor(self, map_data, mx, my):
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if map_data[my+dy, mx+dx] == -1:  # -1 indicates unknown space
                    return True
        return False

    def _is_valid_boundary(self, map_data, mx, my, height, width):
        for radius in range(2, 5):
            has_obstacle = False
            
            for dy in range(-radius, radius+1):
                for dx in range(-radius, radius+1):
                    if (0 <= my+dy < height and 0 <= mx+dx < width):
                        if map_data[my+dy, mx+dx] > 50:  # Occupied space
                            if radius == 2:
                                return False  # Too close to obstacle
                            has_obstacle = True
            
            if has_obstacle:
                return True  # Found a good boundary point
        return False