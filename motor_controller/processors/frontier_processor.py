import numpy as np
from queue import Queue
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Frontier:
    center: Tuple[int, int]  # (x, y) in map coordinates
    size: int  # Number of frontier cells
    distance: float  # Distance from robot

class FrontierProcessor:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.robot_position = (0, 0)
        
    def update_map(self, map_msg):
        """Update stored map data."""
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.map_info = map_msg.info
        
    def update_robot_position(self, x, y):
        """Update robot position in map coordinates."""
        self.robot_position = (
            int((x - self.map_info.origin.position.x) / self.map_info.resolution),
            int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        )
        
    def is_frontier_point(self, x, y) -> bool:
        """Check if a point is a frontier cell (unexplored cell next to free space)."""
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
        
    def find_frontiers(self, min_size=10) -> List[Frontier]:
        """Find all frontier regions in the map."""
        if self.map_data is None:
            return []
            
        frontiers = []
        visited = np.zeros_like(self.map_data, dtype=bool)
        
        # Scan the entire map for frontier points
        for y in range(self.map_data.shape[0]):
            for x in range(self.map_data.shape[1]):
                if not visited[y, x] and self.is_frontier_point(x, y):
                    # Found a new frontier point, flood fill to find the region
                    frontier = self._flood_fill_frontier(x, y, visited)
                    
                    if len(frontier) >= min_size:
                        # Calculate center of mass
                        center_x = sum(p[0] for p in frontier) / len(frontier)
                        center_y = sum(p[1] for p in frontier) / len(frontier)
                        
                        # Calculate distance from robot
                        dx = center_x - self.robot_position[0]
                        dy = center_y - self.robot_position[1]
                        distance = np.sqrt(dx*dx + dy*dy)
                        
                        frontiers.append(Frontier(
                            center=(center_x, center_y),
                            size=len(frontier),
                            distance=distance
                        ))
                        
        return frontiers
        
    def _flood_fill_frontier(self, start_x, start_y, visited) -> List[Tuple[int, int]]:
        """Flood fill algorithm to find connected frontier regions."""
        frontier_points = []
        queue = Queue()
        queue.put((start_x, start_y))
        
        while not queue.empty():
            x, y = queue.get()
            
            if visited[y, x]:
                continue
                
            visited[y, x] = True
            
            if self.is_frontier_point(x, y):
                frontier_points.append((x, y))
                
                # Add neighbors to queue
                neighbors = [
                    (x+1, y), (x-1, y), (x, y+1), (x, y-1),
                    (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)
                ]
                
                for nx, ny in neighbors:
                    if (0 <= nx < self.map_data.shape[1] and 
                        0 <= ny < self.map_data.shape[0] and 
                        not visited[ny, nx]):
                        queue.put((nx, ny))
                        
        return frontier_points
        
    def get_best_frontier(self) -> Tuple[float, float]:
        """Get the best frontier to explore next."""
        frontiers = self.find_frontiers()
        if not frontiers:
            return None
            
        # Sort by distance and size
        frontiers.sort(key=lambda f: (f.distance, -f.size))
        best = frontiers[0]
        
        # Convert back to world coordinates
        world_x = best.center[0] * self.map_info.resolution + self.map_info.origin.position.x
        world_y = best.center[1] * self.map_info.resolution + self.map_info.origin.position.y
        
        return (world_x, world_y) 