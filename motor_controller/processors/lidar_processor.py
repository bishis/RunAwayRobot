import numpy as np

class LidarProcessor:
    """Processes LIDAR data for obstacle detection and navigation."""
    
    def __init__(self, safety_radius=0.3, detection_distance=0.5):
        self.SAFETY_RADIUS = safety_radius
        self.DETECTION_DISTANCE = detection_distance
        
        # Define sectors for different directions (RPLIDAR A1 mounted backwards)
        self.SECTOR_SIZE = 30  # degrees
        self.sectors = {
            'front': (180, 210),    # 180° to 210° is now front
            'front_left': (210, 240),
            'left': (260, 280),     # ~270° is now left
            'rear': (0, 20),        # 0° is now rear
            'right': (80, 100)      # ~90° is now right
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
        left_dist = sector_data['left']['min_distance']
        right_dist = sector_data['right']['min_distance']
        
        # If extremely close to front wall, reverse
        if front_dist < self.SAFETY_RADIUS:
            return 'reverse'
            
        # If approaching wall, decide which way to turn
        if front_dist < self.DETECTION_DISTANCE:
            # Turn towards the side with more space
            if left_dist > right_dist:
                return 'turn_left'
            else:
                return 'turn_right'
        
        # Wall following behavior
        target_wall_distance = 0.5  # Desired distance from wall
        
        # If we're too far from both walls, move forward
        if left_dist > target_wall_distance and right_dist > target_wall_distance:
            return 'forward'
            
        # Follow the nearest wall
        if left_dist <= right_dist:
            # Following left wall
            if left_dist < target_wall_distance * 0.8:  # Too close to left wall
                return 'turn_right'
            elif left_dist > target_wall_distance * 1.2:  # Too far from left wall
                return 'turn_left'
        else:
            # Following right wall
            if right_dist < target_wall_distance * 0.8:  # Too close to right wall
                return 'turn_left'
            elif right_dist > target_wall_distance * 1.2:  # Too far from right wall
                return 'turn_right'
        
        return 'forward' 