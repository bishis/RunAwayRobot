import numpy as np

class LidarProcessor:
    """Processes LIDAR data for obstacle detection and navigation."""
    
    def __init__(self, safety_radius=0.3, detection_distance=0.5):
        self.SAFETY_RADIUS = safety_radius
        self.DETECTION_DISTANCE = detection_distance
        
        # Redefine sectors for better coverage
        self.SECTOR_SIZE = 45  # Increased from 30
        self.sectors = {
            'front': (165, 195),      # 180° ± 15°
            'front_left': (195, 240),  # Wider front-left sector
            'left': (240, 300),       # Wider left sector
            'rear': (345, 15),        # Rear sector
            'right': (60, 120),       # Wider right sector
            'front_right': (120, 165)  # Added front-right sector
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
        """Determine navigation command based on LIDAR data."""
        if not sector_data:
            return 'stop'
            
        front_dist = sector_data['front']['min_distance']
        front_left_dist = sector_data['front_left']['min_distance']
        front_right_dist = sector_data['front_right']['min_distance']
        left_dist = sector_data['left']['min_distance']
        right_dist = sector_data['right']['min_distance']
        
        # Emergency stop if too close
        if front_dist < self.SAFETY_RADIUS:
            return 'reverse'
            
        # Wall following parameters
        target_wall_distance = 0.4  # Slightly reduced
        wall_follow_threshold = 0.1  # Added threshold for smoother following
        
        # If approaching obstacle, make smoother turns
        if front_dist < self.DETECTION_DISTANCE:
            if front_left_dist > front_right_dist:
                return 'turn_left_gentle'
            else:
                return 'turn_right_gentle'
        
        # Wall following behavior
        if min(left_dist, right_dist) < target_wall_distance + wall_follow_threshold:
            # Following closest wall
            if left_dist <= right_dist:
                if left_dist < target_wall_distance - wall_follow_threshold:
                    return 'turn_right_gentle'
                elif left_dist > target_wall_distance + wall_follow_threshold:
                    return 'turn_left_gentle'
            else:
                if right_dist < target_wall_distance - wall_follow_threshold:
                    return 'turn_left_gentle'
                elif right_dist > target_wall_distance + wall_follow_threshold:
                    return 'turn_right_gentle'
        
        return 'forward' 