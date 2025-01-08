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
        self.max_angle_deviation = math.pi/3  # Maximum angle deviation from direct path
        
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

    def find_path(self, current_pos, target_pos):
        """Find best path using Monte Carlo sampling."""
        if self.map_data is None or self.map_info is None:
            print("No map data available for path planning")
            return None
        
        try:
            best_path = None
            best_score = float('-inf')
            direct_angle = math.atan2(
                target_pos.y - current_pos.y,
                target_pos.x - current_pos.x
            )
            
            # Generate multiple candidate paths
            for _ in range(self.num_samples):
                path = self._generate_candidate_path(
                    current_pos, target_pos, direct_angle
                )
                if path:
                    score = self._evaluate_path(path, target_pos)
                    if score > best_score:
                        best_score = score
                        best_path = path
            
            if best_path is None:
                print("Could not find valid path")
            
            return best_path
            
        except Exception as e:
            print(f"Error in path planning: {str(e)}")
            return None

    def _generate_candidate_path(self, start_pos, target_pos, direct_angle):
        """Generate a single candidate path."""
        try:
            path = [(start_pos.x, start_pos.y)]
            current_x, current_y = start_pos.x, start_pos.y
            max_steps = 20  # Prevent infinite paths
            min_step_size = self.step_size * 0.5  # Allow smaller steps if needed
            
            for _ in range(max_steps):
                # Calculate distance to target
                dx = target_pos.x - current_x
                dy = target_pos.y - current_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < min_step_size:
                    path.append((target_pos.x, target_pos.y))
                    return path
                    
                # Use smaller steps when close to obstacles
                current_clearance = self._calculate_clearance(current_x, current_y)
                current_step = self.step_size * max(0.5, current_clearance)
                
                # Generate random angle within cone towards target
                target_angle = math.atan2(dy, dx)
                angle = random.uniform(
                    target_angle - self.max_angle_deviation,
                    target_angle + self.max_angle_deviation
                )
                
                # Calculate next point
                next_x = current_x + current_step * math.cos(angle)
                next_y = current_y + current_step * math.sin(angle)
                
                # Check if point is valid
                if self._is_path_segment_valid(current_x, current_y, next_x, next_y):
                    path.append((next_x, next_y))
                    current_x, current_y = next_x, next_y
                else:
                    return None
            
            return None  # Path too long
            
        except Exception as e:
            print(f"Error generating path: {str(e)}")
            return None

    def _is_path_segment_valid(self, x1, y1, x2, y2):
        """Check if path segment is collision-free."""
        try:
            # Check multiple points along the segment
            steps = int(math.sqrt((x2-x1)**2 + (y2-y1)**2) / (self.safety_radius/2))
            steps = max(steps, 5)  # Minimum number of checks
            
            for i in range(steps + 1):
                t = i / steps
                x = x1 + t*(x2-x1)
                y = y1 + t*(y2-y1)
                
                if not self.is_valid_point(x, y):
                    return False
            
            return True
            
        except Exception as e:
            print(f"Error checking path segment: {str(e)}")
            return False

    def _evaluate_path(self, path, target_pos):
        """Score a path based on multiple criteria."""
        if len(path) < 2:
            return float('-inf')
            
        # Path length score (shorter is better)
        length = sum(
            math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            for p1, p2 in zip(path[:-1], path[1:])
        )
        length_score = 1.0 / (1.0 + length)
        
        # Clearance score (further from obstacles is better)
        clearance_score = sum(
            self._calculate_clearance(x, y)
            for x, y in path
        ) / len(path)
        
        # Directness score (closer to straight line is better)
        directness = self._calculate_directness(path, target_pos)
        
        # Combine scores with weights
        return (0.4 * length_score + 
                0.4 * clearance_score + 
                0.2 * directness)

    def _calculate_clearance(self, x, y):
        """Calculate clearance from obstacles."""
        mx, my = self.world_to_map(x, y)
        if mx is None or my is None:
            return 0.0
            
        min_distance = float('inf')
        search_radius = int(self.safety_radius * 2 / self.map_info.resolution)
        
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_info.width and 
                    0 <= check_y < self.map_info.height):
                    if self.map_data[check_y, check_x] > 50:
                        dist = math.sqrt(dx*dx + dy*dy) * self.map_info.resolution
                        min_distance = min(min_distance, dist)
        
        return 1.0 if min_distance == float('inf') else min(1.0, min_distance / (self.safety_radius * 2))

    def _calculate_directness(self, path, target_pos):
        """Calculate how direct the path is to the target."""
        if len(path) < 2:
            return 0.0
            
        # Calculate total path length
        path_length = sum(
            math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            for p1, p2 in zip(path[:-1], path[1:])
        )
        
        # Calculate direct distance
        direct_distance = math.sqrt(
            (target_pos.x - path[0][0])**2 + 
            (target_pos.y - path[0][1])**2
        )
        
        return direct_distance / path_length if path_length > 0 else 0.0 