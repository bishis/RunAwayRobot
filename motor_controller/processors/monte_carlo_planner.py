import math
import random
from geometry_msgs.msg import Point

class MonteCarloPlanner:
    def __init__(self, robot_radius, safety_margin):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.min_waypoint_spacing = robot_radius * 3
        
    def find_best_path(self, current_x, current_y, target_x, target_y, map_data, map_info, is_valid_point, num_samples=200):
        """Find best path using Monte Carlo sampling."""
        best_point = None
        best_score = float('-inf')
        
        # Sample points in a circle around current position
        for _ in range(num_samples):
            # Random angle and distance
            angle = random.uniform(0, 2 * math.pi)
            # Vary distance between 0.3 and 1.0 meters
            distance = random.uniform(0.3, 1.0)
            
            # Generate sample point
            sample_x = current_x + distance * math.cos(angle)
            sample_y = current_y + distance * math.sin(angle)
            
            if not is_valid_point(sample_x, sample_y):
                continue
                
            # Score based on multiple factors
            # 1. Distance to target
            dx_target = target_x - sample_x
            dy_target = target_y - sample_y
            dist_to_target = math.sqrt(dx_target*dx_target + dy_target*dy_target)
            target_score = 1.0 / (1.0 + dist_to_target)
            
            # 2. Clearance from obstacles
            mx, my = self._world_to_map(sample_x, sample_y, map_info)
            if mx is None:
                continue
            
            clearance = self._calculate_clearance(mx, my, map_data, map_info)
            
            # 3. Progress towards goal
            progress = 1.0 - (dist_to_target / math.sqrt((target_x-current_x)**2 + (target_y-current_y)**2))
            
            # Combine scores with weights
            total_score = (
                0.4 * target_score +
                0.4 * clearance +
                0.2 * progress
            )
            
            if total_score > best_score:
                best_score = total_score
                best_point = (sample_x, sample_y)
        
        return best_point if best_point else None

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