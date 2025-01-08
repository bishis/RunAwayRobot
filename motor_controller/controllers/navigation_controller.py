from typing import Tuple, List, Optional
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import math

class NavigationController:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.current_pose = None
        self.goal = None
        self.path = []
        
        # Navigation parameters
        self.min_frontier_size = 10
        self.robot_radius = 0.2  # meters
        self.safety_margin = 0.3  # meters
        self.goal_tolerance = 0.2  # meters
        
    def update_map(self, map_msg: OccupancyGrid):
        """Update stored map data."""
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.map_info = map_msg.info
        
    def update_pose(self, pose: Pose):
        """Update robot's current pose."""
        self.current_pose = pose
        
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map cell coordinates."""
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (mx, my)
        
    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map cell coordinates to world coordinates."""
        x = mx * self.map_info.resolution + self.map_info.origin.position.x
        y = my * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)
        
    def find_frontiers(self) -> List[Tuple[int, int]]:
        """Find frontier cells (unexplored areas next to known areas)."""
        if self.map_data is None:
            return []
            
        frontiers = []
        height, width = self.map_data.shape
        
        # Find frontier cells
        for y in range(height):
            for x in range(width):
                if self.is_frontier_cell(x, y):
                    frontiers.append((x, y))
                    
        return frontiers
        
    def is_frontier_cell(self, x: int, y: int) -> bool:
        """Check if cell is a frontier (unknown cell next to free space)."""
        if not (0 <= x < self.map_data.shape[1] and 0 <= y < self.map_data.shape[0]):
            return False
            
        # Cell must be unknown (-1)
        if self.map_data[y, x] != -1:
            return False
            
        # Must have at least one free neighbor
        neighbors = [
            (x+1, y), (x-1, y), (x, y+1), (x, y-1),
            (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)
        ]
        
        for nx, ny in neighbors:
            if (0 <= nx < self.map_data.shape[1] and 
                0 <= ny < self.map_data.shape[0] and 
                self.map_data[ny, nx] == 0):  # Free space
                return True
        return False
        
    def find_best_frontier(self) -> Optional[Tuple[float, float]]:
        """Find best frontier to explore next."""
        if self.current_pose is None or self.map_data is None:
            return None
            
        frontiers = self.find_frontiers()
        if not frontiers:
            return None
            
        # Convert current position to map coordinates
        current_x, current_y = self.world_to_map(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        
        # Find closest frontier cluster
        clusters = self.cluster_frontiers(frontiers)
        if not clusters:
            return None
            
        # Score each cluster based on size and distance
        best_score = float('-inf')
        best_cluster = None
        
        for cluster in clusters:
            if len(cluster) < self.min_frontier_size:
                continue
                
            # Calculate cluster center
            center_x = sum(x for x, _ in cluster) / len(cluster)
            center_y = sum(y for _, y in cluster) / len(cluster)
            
            # Calculate distance
            dx = center_x - current_x
            dy = center_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Score based on size and inverse distance
            score = len(cluster) / (distance + 1)
            
            if score > best_score:
                best_score = score
                best_cluster = cluster
                
        if best_cluster is None:
            return None
            
        # Convert cluster center to world coordinates
        center_x = sum(x for x, _ in best_cluster) / len(best_cluster)
        center_y = sum(y for _, y in best_cluster) / len(best_cluster)
        world_x, world_y = self.map_to_world(int(center_x), int(center_y))
        
        return (world_x, world_y)
        
    def cluster_frontiers(self, frontiers: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        """Group frontier cells into clusters."""
        if not frontiers:
            return []
            
        clusters = []
        visited = set()
        
        for cell in frontiers:
            if cell in visited:
                continue
                
            # Start new cluster
            cluster = []
            queue = [cell]
            
            while queue:
                current = queue.pop(0)
                if current in visited:
                    continue
                    
                visited.add(current)
                cluster.append(current)
                
                # Check neighbors
                x, y = current
                neighbors = [
                    (x+1, y), (x-1, y), (x, y+1), (x, y-1),
                    (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)
                ]
                
                for nx, ny in neighbors:
                    if (nx, ny) in frontiers and (nx, ny) not in visited:
                        queue.append((nx, ny))
                        
            clusters.append(cluster)
            
        return clusters
        
    def is_path_clear(self, start: Tuple[float, float], goal: Tuple[float, float]) -> bool:
        """Check if there's a clear path between two points."""
        if self.map_data is None:
            return False
            
        # Convert to map coordinates
        start_map = self.world_to_map(start[0], start[1])
        goal_map = self.world_to_map(goal[0], goal[1])
        
        # Use Bresenham's line algorithm to check path
        x0, y0 = start_map
        x1, y1 = goal_map
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2
        
        safety_cells = int(self.safety_margin / self.map_info.resolution)
        
        for _ in range(n):
            # Check if current cell and surrounding cells are safe
            for dx in range(-safety_cells, safety_cells + 1):
                for dy in range(-safety_cells, safety_cells + 1):
                    check_x, check_y = x + dx, y + dy
                    if (0 <= check_x < self.map_data.shape[1] and 
                        0 <= check_y < self.map_data.shape[0]):
                        if self.map_data[check_y, check_x] > 50:  # Occupied
                            return False
            
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
                
        return True 