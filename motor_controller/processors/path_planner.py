import numpy as np
import math
import random
from geometry_msgs.msg import Point

class PathPlanner:
    def __init__(self, safety_radius=0.25, num_samples=20, step_size=0.2):
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

    def find_next_point(self, current_pos, target_pos, scan_data):
        """Find next best point to move to."""
        if not self.map_data is not None:
            return None
            
        best_point = None
        best_score = float('-inf')
        
        # Calculate base direction to target
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        target_angle = math.atan2(dy, dx)
        
        # Generate sample points in an arc
        for _ in range(self.num_samples):
            # Sample angle in forward 180-degree arc
            angle_offset = random.uniform(-math.pi/2, math.pi/2)
            sample_angle = target_angle + angle_offset
            
            # Generate point at step_size distance
            sample_x = current_pos.x + self.step_size * math.cos(sample_angle)
            sample_y = current_pos.y + self.step_size * math.sin(sample_angle)
            
            if not self.is_valid_point(sample_x, sample_y):
                continue
                
            # Score the point based on multiple factors
            score = self.score_point(
                sample_x, sample_y,
                current_pos, target_pos,
                scan_data, sample_angle
            )
            
            if score > best_score:
                best_score = score
                best_point = Point(x=sample_x, y=sample_y, z=0.0)
        
        return best_point

    def score_point(self, x, y, current_pos, target_pos, scan_data, angle):
        """Score a potential point based on multiple factors."""
        # Distance to target (prefer points closer to target)
        dx_target = target_pos.x - x
        dy_target = target_pos.y - y
        dist_to_target = math.sqrt(dx_target*dx_target + dy_target*dy_target)
        target_score = 1.0 / (1.0 + dist_to_target)
        
        # Distance from current position (prefer points we can reach)
        dx_current = x - current_pos.x
        dy_current = y - current_pos.y
        dist_from_current = math.sqrt(dx_current*dx_current + dy_current*dy_current)
        distance_score = 1.0 if dist_from_current <= self.step_size else 0.0
        
        # Clearance from obstacles
        clearance_score = self.calculate_clearance_score(x, y)
        
        # Direction score (prefer points in general direction of target)
        target_angle = math.atan2(target_pos.y - current_pos.y, 
                                target_pos.x - current_pos.x)
        angle_diff = abs(target_angle - angle)
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        direction_score = 1.0 - (abs(angle_diff) / math.pi)
        
        # Combine scores with weights
        total_score = (
            0.4 * target_score +
            0.3 * clearance_score +
            0.2 * direction_score +
            0.1 * distance_score
        )
        
        return total_score

    def calculate_clearance_score(self, x, y):
        """Calculate how far the point is from obstacles."""
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
                    if self.map_data[check_y, check_x] > 50:  # Occupied
                        dist = math.sqrt(dx*dx + dy*dy) * self.map_info.resolution
                        min_distance = min(min_distance, dist)
        
        if min_distance == float('inf'):
            return 1.0
        
        # Convert distance to score (0 to 1)
        return min(1.0, min_distance / (self.safety_radius * 2)) 