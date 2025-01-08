import numpy as np
import math
import random
from geometry_msgs.msg import Point

class PathPlanner:
    def __init__(self, safety_radius=0.25, num_samples=200, step_size=0.2):
        self.robot_radius = 0.17
        self.safety_margin = 0.10
        self.safety_radius = safety_radius
        self.num_samples = num_samples
        self.step_size = step_size
        self.map_data = None
        self.map_info = None

    def update_map(self, map_data, map_info):
        """Update the map data used for planning."""
        self.map_data = map_data
        self.map_info = map_info
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates."""
        if not self.map_info:
            return None, None
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return mx, my
        return None, None

    def is_valid_point(self, x, y):
        """Check if point is valid and free in map."""
        mx, my = self.world_to_map(x, y)
        if mx is None or my is None:
            return False
            
        # Check area around point for obstacles
        radius_cells = int(self.safety_radius / self.map_info.resolution)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_info.width and 
                    0 <= check_y < self.map_info.height):
                    if self.map_data[check_y, check_x] > 50:  # Occupied
                        return False
        return True

    def find_best_path(self, current_pos, target_pos):
        """Use Monte Carlo sampling to find the best path."""
        if not self.map_data is not None:
            return None

        best_path = None
        best_score = float('-inf')
        
        # Generate multiple random paths
        for _ in range(self.num_samples):
            path = self._generate_random_path(current_pos, target_pos)
            if path:
                score = self._evaluate_path(path, target_pos)
                if score > best_score:
                    best_score = score
                    best_path = path

        return best_path

    def _generate_random_path(self, start_pos, target_pos):
        """Generate a random path using RRT-like approach."""
        path = [(start_pos.x, start_pos.y)]
        current = path[0]
        max_iterations = 50
        
        for _ in range(max_iterations):
            # Bias towards target (80% chance to sample towards target)
            if random.random() < 0.8:
                sample_x = target_pos.x
                sample_y = target_pos.y
            else:
                # Random sample within reasonable bounds
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(0, self.step_size * 2)
                sample_x = current[0] + distance * math.cos(angle)
                sample_y = current[1] + distance * math.sin(angle)

            # Find step towards sample
            dx = sample_x - current[0]
            dy = sample_y - current[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > self.step_size:
                # Normalize to step size
                dx = dx * self.step_size / dist
                dy = dy * self.step_size / dist
            
            new_x = current[0] + dx
            new_y = current[1] + dy

            # Check if new point is valid
            if self._is_path_clear(current[0], current[1], new_x, new_y):
                current = (new_x, new_y)
                path.append(current)
                
                # Check if we're close enough to target
                dist_to_target = math.sqrt(
                    (target_pos.x - new_x)**2 + 
                    (target_pos.y - new_y)**2
                )
                if dist_to_target < self.step_size:
                    path.append((target_pos.x, target_pos.y))
                    return path
            else:
                # If path is blocked, try a different direction
                continue
                
        return None

    def _is_path_clear(self, x1, y1, x2, y2):
        """Check if path between two points is clear using Bresenham's line algorithm."""
        # Convert to map coordinates
        mx1, my1 = self.world_to_map(x1, y1)
        mx2, my2 = self.world_to_map(x2, y2)
        
        if mx1 is None or mx2 is None:
            return False

        points = self._get_line_points(mx1, my1, mx2, my2)
        check_radius = int(self.safety_radius / self.map_info.resolution)

        for px, py in points:
            # Check circle around point
            for dx in range(-check_radius, check_radius + 1):
                for dy in range(-check_radius, check_radius + 1):
                    if dx*dx + dy*dy <= check_radius*check_radius:
                        check_x = px + dx
                        check_y = py + dy
                        if (0 <= check_x < self.map_info.width and 
                            0 <= check_y < self.map_info.height):
                            if self.map_data[check_y, check_x] > 50:
                                return False
        return True

    def _get_line_points(self, x1, y1, x2, y2):
        """Get points along line using Bresenham's algorithm."""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        points.append((x, y))
        return points

    def _evaluate_path(self, path, target_pos):
        """Score a path based on multiple criteria."""
        if not path:
            return float('-inf')

        # Path length score (shorter is better)
        length = sum(math.sqrt(
            (path[i+1][0] - path[i][0])**2 + 
            (path[i+1][1] - path[i][1])**2
        ) for i in range(len(path)-1))
        length_score = 1.0 / (1.0 + length)

        # Clearance score (further from obstacles is better)
        clearance_scores = []
        for x, y in path:
            mx, my = self.world_to_map(x, y)
            if mx is not None:
                clearance = self._calculate_clearance(mx, my)
                clearance_scores.append(clearance)
        avg_clearance = sum(clearance_scores) / len(clearance_scores) if clearance_scores else 0

        # Directness score (closer to straight line is better)
        path_length = length
        straight_length = math.sqrt(
            (path[-1][0] - path[0][0])**2 + 
            (path[-1][1] - path[0][1])**2
        )
        directness = straight_length / path_length if path_length > 0 else 0

        # Combine scores with weights
        return (0.4 * length_score + 
                0.4 * avg_clearance + 
                0.2 * directness)

    def _calculate_clearance(self, mx, my):
        """Calculate clearance from obstacles at a point."""
        check_radius = int(self.safety_radius * 2 / self.map_info.resolution)
        min_distance = float('inf')

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_info.width and 
                    0 <= check_y < self.map_info.height):
                    if self.map_data[check_y, check_x] > 50:
                        dist = math.sqrt(dx*dx + dy*dy) * self.map_info.resolution
                        min_distance = min(min_distance, dist)

        if min_distance == float('inf'):
            return 1.0
        return min(1.0, min_distance / (self.safety_radius * 2)) 