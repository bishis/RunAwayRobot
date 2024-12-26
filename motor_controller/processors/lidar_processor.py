import numpy as np

class LidarProcessor:
    """Processes LIDAR data for obstacle detection and navigation."""
    
    def __init__(self, safety_radius=0.3, detection_distance=0.5):
        self.SAFETY_RADIUS = safety_radius
        self.DETECTION_DISTANCE = detection_distance
        
        # Define sectors for RPLIDAR A1 (0 degrees is front)
        self.sectors = {
            'front': (350, 10),     # Front sector ±10 degrees
            'front_left': (10, 45),  # Front-left sector
            'front_right': (315, 350),  # Front-right sector
            'left': (45, 135),      # Left sector
            'right': (225, 315)     # Right sector
        }
    
    def process_scan(self, ranges):
        """Process full 360° LIDAR scan and return processed data."""
        if not ranges:
            return None
            
        num_readings = len(ranges)
        angles = np.linspace(0, 360, num_readings, endpoint=False)
        
        # Convert ranges to numpy array for efficient processing
        ranges_array = np.array(ranges)
        
        # Filter out invalid readings and zeros
        valid_mask = (np.isfinite(ranges_array)) & (ranges_array > 0.1)
        valid_ranges = ranges_array[valid_mask]
        valid_angles = angles[valid_mask]
        
        sector_data = {}
        for sector_name, (start, end) in self.sectors.items():
            # Handle wraparound for front sector
            if start > end:  # For sectors that cross 0°
                mask = (valid_angles >= start) | (valid_angles <= end)
            else:
                mask = (valid_angles >= start) & (valid_angles <= end)
            
            sector_ranges = valid_ranges[mask]
            
            if len(sector_ranges) > 0:
                sector_data[sector_name] = {
                    'min_distance': float(np.min(sector_ranges)),
                    'mean_distance': float(np.mean(sector_ranges)),
                    'readings': len(sector_ranges)
                }
            else:
                sector_data[sector_name] = {
                    'min_distance': float('inf'),
                    'mean_distance': float('inf'),
                    'readings': 0
                }
        
        return sector_data

    def get_navigation_command(self, sector_data):
        """Determine if path is clear for movement."""
        if not sector_data:
            return 'stop'
            
        front_dist = sector_data['front']['min_distance']
        front_left_dist = sector_data['front_left']['min_distance']
        front_right_dist = sector_data['front_right']['min_distance']
        
        # Emergency stop if too close to obstacles
        if front_dist < self.SAFETY_RADIUS or \
           front_left_dist < self.SAFETY_RADIUS or \
           front_right_dist < self.SAFETY_RADIUS:
            return False
            
        # Check if path is clear
        if front_dist < self.DETECTION_DISTANCE:
            return False
            
        return True 