import math
from geometry_msgs.msg import Point
import rclpy

class WaypointGenerator:
    def __init__(self, robot_radius, safety_margin, num_waypoints):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.num_waypoints = num_waypoints
        self.min_waypoint_spacing = robot_radius * 3
        
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
        
        for my in range(1, height-1):
            for mx in range(1, width-1):
                if map_data[my, mx] == 0:  # Free space
                    if self._has_unknown_neighbor(map_data, mx, my):
                        wx, wy = map_to_world(mx, my)
                        if wx is not None and wy is not None:
                            frontiers.append((wx, wy))
        return frontiers

    def _find_boundary_points(self, map_data, map_info, map_to_world):
        boundaries = []
        height, width = map_data.shape
        
        for my in range(1, height-1):
            for mx in range(1, width-1):
                if map_data[my, mx] == 0:
                    if self._is_valid_boundary(map_data, mx, my, height, width):
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
                if map_data[my+dy, mx+dx] == -1:
                    return True
        return False

    def _is_valid_boundary(self, map_data, mx, my, height, width):
        for radius in range(2, 5):
            has_obstacle = False
            too_close = False
            
            for dy in range(-radius, radius+1):
                for dx in range(-radius, radius+1):
                    if (0 <= my+dy < height and 0 <= mx+dx < width):
                        if map_data[my+dy, mx+dx] > 50:
                            if radius == 2:
                                return False
                            has_obstacle = True
            
            if has_obstacle:
                return True
        return False 