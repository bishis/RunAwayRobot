import numpy as np

class LidarProcessor:
    """Processes LIDAR data for obstacle detection and navigation."""
    
    def __init__(self, safety_radius=0.3, detection_distance=0.5):
        self.SAFETY_RADIUS = safety_radius
        self.DETECTION_DISTANCE = detection_distance
        
        # Define sectors for RPLIDAR A1 (0 degrees is front)
        # Adjusted sectors to be more precise
        self.sectors = {
            'front': (355, 5),       # Narrower front sector (±5 degrees)
            'front_left': (5, 60),   # Wider front-left sector
            'front_right': (300, 355), # Wider front-right sector
            'left': (60, 120),
            'right': (240, 300)
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
        
        # Debug print for raw LIDAR data
        print("\nRaw LIDAR data sample:")
        for i in range(0, len(ranges), len(ranges)//8):  # Print 8 sample points
            print(f"Angle {i/len(ranges)*360:.1f}°: {ranges[i]:.2f}m")
        
        sector_data = {}
        for sector_name, (start, end) in self.sectors.items():
            # Handle wraparound for front sector
            if start > end:  # For sectors that cross 0°
                mask = (valid_angles >= start) | (valid_angles <= end)
            else:
                mask = (valid_angles >= start) & (valid_angles <= end)
            
            sector_ranges = valid_ranges[mask]
            
            if len(sector_ranges) > 0:
                min_dist = float(np.min(sector_ranges))
                mean_dist = float(np.mean(sector_ranges))
                sector_data[sector_name] = {
                    'min_distance': min_dist,
                    'mean_distance': mean_dist,
                    'readings': len(sector_ranges)
                }
                print(f"\nSector {sector_name}:")
                print(f"  Range of values: {np.min(sector_ranges):.2f}m to {np.max(sector_ranges):.2f}m")
                print(f"  Number of readings: {len(sector_ranges)}")
            else:
                sector_data[sector_name] = {
                    'min_distance': float('inf'),
                    'mean_distance': float('inf'),
                    'readings': 0
                }
                print(f"\nSector {sector_name}: No valid readings")
        
        return sector_data

    def get_navigation_command(self, sector_data):
        """Determine if path is clear for movement."""
        if not sector_data:
            print("No sector data available")
            return 'stop'
            
        front_dist = sector_data['front']['min_distance']
        front_left_dist = sector_data['front_left']['min_distance']
        front_right_dist = sector_data['front_right']['min_distance']
        
        print(f"\nLIDAR Distances:")
        print(f"  Front: {front_dist:.2f}m (readings: {sector_data['front']['readings']})")
        print(f"  Front-Left: {front_left_dist:.2f}m (readings: {sector_data['front_left']['readings']})")
        print(f"  Front-Right: {front_right_dist:.2f}m (readings: {sector_data['front_right']['readings']})")
        
        # Adjusted thresholds
        if front_dist < self.SAFETY_RADIUS * 1.2 or \
           front_left_dist < self.SAFETY_RADIUS or \
           front_right_dist < self.SAFETY_RADIUS:
            print(f"Emergency stop - Obstacle detected:")
            print(f"  Safety radius: {self.SAFETY_RADIUS}m")
            print(f"  Front distance: {front_dist:.2f}m")
            return False
            
        if front_dist < self.DETECTION_DISTANCE:
            print(f"Path blocked - Obstacle at {front_dist:.2f}m")
            return False
            
        print("Path is clear")
        return True 