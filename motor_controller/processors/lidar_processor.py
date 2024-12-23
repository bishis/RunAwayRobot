class LidarProcessor:
    """Processes LIDAR data for obstacle detection."""
    
    def __init__(self, safety_radius=0.3, detection_distance=0.5):
        self.SAFETY_RADIUS = safety_radius
        self.DETECTION_DISTANCE = detection_distance
        
    def process_scan(self, ranges):
        """Process LIDAR ranges and return minimum front distance."""
        num_readings = len(ranges)
        front_start = num_readings // 3
        front_end = 2 * num_readings // 3
        front_distances = ranges[front_start:front_end]
        return min(front_distances) 