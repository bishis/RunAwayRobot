import math
from geometry_msgs.msg import Point
import rclpy
import random
from .monte_carlo_planner import MonteCarloPlanner

class WaypointGenerator:
    def __init__(self, robot_radius, safety_margin, num_waypoints):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.num_waypoints = num_waypoints
        self.min_waypoint_spacing = robot_radius * 3
        self.planner = MonteCarloPlanner(robot_radius, safety_margin)
        
    def generate_waypoints(self, current_pose, map_data, map_info, is_valid_point, map_to_world):
        """Generate waypoints using Monte Carlo sampling."""
        if not current_pose or not map_info:
            return []
        
        points = []
        x = current_pose.position.x
        y = current_pose.position.y
        
        # Find frontiers and boundaries
        frontiers = self._find_unexplored_frontier(map_data, map_info, map_to_world)
        boundaries = self._find_boundary_points(map_data, map_info, map_to_world)
        
        # Sort by distance
        candidates = []
        for wx, wy in frontiers:
            candidates.append(('frontier', wx, wy))
        for wx, wy in boundaries:
            candidates.append(('boundary', wx, wy))
        
        candidates.sort(key=lambda p: math.sqrt((p[1]-x)**2 + (p[2]-y)**2))
        
        # Generate path using Monte Carlo for each waypoint
        while len(points) < self.num_waypoints and candidates:
            target_type = 'frontier' if len(points) % 2 == 0 else 'boundary'
            
            for i, (point_type, target_x, target_y) in enumerate(candidates):
                if point_type == target_type:
                    # Find best intermediate point using Monte Carlo
                    best_point = self.planner.find_best_path(
                        x, y, target_x, target_y,
                        map_data, map_info, is_valid_point
                    )
                    
                    if best_point:
                        point = Point(x=best_point[0], y=best_point[1], z=0.0)
                        if not self._is_point_too_close(point.x, point.y, points):
                            points.append(point)
                            candidates.pop(i)
                            # Update current position for next iteration
                            x, y = best_point
                            break
            else:
                target_type = 'boundary' if target_type == 'frontier' else 'frontier'
                continue
        
        return points

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

    def _calculate_clearance(self, mx, my, map_data, map_info):
        """Calculate clearance score for a point."""
        clearance = 0
        max_check = 5  # Check up to 5 cells away
        
        for r in range(1, max_check + 1):
            clear_cells = 0
            total_cells = 0
            
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx*dx + dy*dy <= r*r:  # Circular check
                        check_x = mx + dx
                        check_y = my + dy
                        if (0 <= check_x < map_data.shape[1] and 
                            0 <= check_y < map_data.shape[0]):
                            total_cells += 1
                            if map_data[check_y, check_x] == 0:  # Free space
                                clear_cells += 1
            
            if total_cells > 0:
                clearance += (clear_cells / total_cells) * (max_check - r + 1)
        
        return clearance / sum(range(1, max_check + 1))

    def _world_to_map(self, x, y, map_info):
        """Convert world coordinates to map coordinates."""
        mx = int((x - map_info.origin.position.x) / map_info.resolution)
        my = int((y - map_info.origin.position.y) / map_info.resolution)
        if 0 <= mx < map_info.width and 0 <= my < map_info.height:
            return mx, my
        return None, None

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