#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Point
import random
import math

class MonteCarloPathPlanner:
    def __init__(self, safety_radius=0.5, max_iterations=1000):
        self.safety_radius = safety_radius
        self.max_iterations = max_iterations
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        
    def set_map(self, map_data, resolution, origin, width, height):
        """Update map data."""
        self.map_data = map_data
        self.map_resolution = resolution
        self.map_origin = origin
        self.map_width = width
        self.map_height = height
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map indices."""
        if not self.map_data:
            return None, None
            
        mx = int((x - self.map_origin.position.x) / self.map_resolution)
        my = int((y - self.map_origin.position.y) / self.map_resolution)
        
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            return mx, my
        return None, None
    
    def map_to_world(self, mx, my):
        """Convert map indices to world coordinates."""
        x = mx * self.map_resolution + self.map_origin.position.x
        y = my * self.map_resolution + self.map_origin.position.y
        return x, y
    
    def is_valid_point(self, x, y):
        """Check if point is valid and obstacle-free."""
        mx, my = self.world_to_map(x, y)
        if mx is None or my is None:
            return False
            
        # Check point and surrounding area for obstacles
        radius_cells = int(self.safety_radius / self.map_resolution)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_width and 
                    0 <= check_y < self.map_height):
                    idx = check_y * self.map_width + check_x
                    if self.map_data[idx] > 50:  # Occupied cell
                        return False
        return True
    
    def find_path(self, start, goal):
        """Find path using Monte Carlo sampling."""
        if not self.map_data:
            return None
            
        # Initialize RRT-like structure
        nodes = [{'point': start, 'parent': None}]
        best_path = None
        best_distance = float('inf')
        
        for _ in range(self.max_iterations):
            # Random sampling with bias towards goal
            if random.random() < 0.2:  # 20% chance to sample goal directly
                sample = goal
            else:
                # Sample random point in direction of goal
                angle = random.uniform(0, 2 * math.pi)
                distance = random.uniform(0, self.safety_radius * 4)
                dx = math.cos(angle) * distance
                dy = math.sin(angle) * distance
                sample = Point(x=start.x + dx, y=start.y + dy, z=0.0)
            
            # Find nearest node
            nearest_idx = self.find_nearest_node(nodes, sample)
            nearest = nodes[nearest_idx]['point']
            
            # Create new node in direction of sample
            direction = self.normalize_vector(
                sample.x - nearest.x,
                sample.y - nearest.y
            )
            step_size = min(0.5, self.distance(nearest, sample))
            new_point = Point(
                x=nearest.x + direction[0] * step_size,
                y=nearest.y + direction[1] * step_size,
                z=0.0
            )
            
            # Check if new point is valid
            if self.is_valid_point(new_point.x, new_point.y):
                nodes.append({
                    'point': new_point,
                    'parent': nearest_idx
                })
                
                # Check if we can reach goal from here
                if self.distance(new_point, goal) < self.safety_radius:
                    path = self.extract_path(nodes, len(nodes) - 1)
                    path_distance = self.path_length(path)
                    
                    if path_distance < best_distance:
                        best_path = path
                        best_distance = path_distance
        
        return best_path
    
    def find_nearest_node(self, nodes, point):
        """Find nearest node in tree."""
        distances = [self.distance(n['point'], point) for n in nodes]
        return distances.index(min(distances))
    
    def distance(self, p1, p2):
        """Calculate Euclidean distance between points."""
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
    
    def normalize_vector(self, x, y):
        """Normalize 2D vector."""
        length = math.sqrt(x*x + y*y)
        if length == 0:
            return (0, 0)
        return (x/length, y/length)
    
    def extract_path(self, nodes, goal_idx):
        """Extract path from node tree."""
        path = []
        current_idx = goal_idx
        
        while current_idx is not None:
            path.append(nodes[current_idx]['point'])
            current_idx = nodes[current_idx]['parent']
        
        return list(reversed(path))
    
    def path_length(self, path):
        """Calculate total path length."""
        if not path:
            return float('inf')
            
        length = 0
        for i in range(len(path) - 1):
            length += self.distance(path[i], path[i+1])
        return length 