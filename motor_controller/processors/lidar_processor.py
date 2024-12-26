import numpy as np

class LidarProcessor:
    """Processes LIDAR data for obstacle detection and navigation."""
    
    def __init__(self, safety_radius=0.3, detection_distance=0.5):
        self.SAFETY_RADIUS = safety_radius
        self.DETECTION_DISTANCE = detection_distance
        
        # Define sectors for RPLIDAR A1 (mounted backwards)
        self.sectors = {
            'front': (150, 210),     # 60 degree front sector
            'front_left': (210, 270), 
            'front_right': (90, 150),
            'left': (270, 330),
            'right': (30, 90)
        }
        
    def get_navigation_command(self, sector_data):
        """Determine navigation command based on LIDAR data."""
        if not sector_data:
            return 'stop'
            
        # Get distances for all sectors
        front_dist = sector_data['front']['min_distance']
        front_left_dist = sector_data['front_left']['min_distance']
        front_right_dist = sector_data['front_right']['min_distance']
        left_dist = sector_data['left']['min_distance']
        right_dist = sector_data['right']['min_distance']
        
        # Emergency stop and reverse if too close to anything in front
        if front_dist < self.SAFETY_RADIUS or \
           front_left_dist < self.SAFETY_RADIUS or \
           front_right_dist < self.SAFETY_RADIUS:
            return 'reverse'
        
        # Obstacle avoidance - check front area
        if front_dist < self.DETECTION_DISTANCE:
            # Find the best direction to turn based on available space
            if front_left_dist > front_right_dist and front_left_dist > self.DETECTION_DISTANCE:
                return 'turn_left'
            elif front_right_dist > self.DETECTION_DISTANCE:
                return 'turn_right'
            else:
                # If both sides are blocked, make a sharp turn to the side with more space
                return 'turn_left' if left_dist > right_dist else 'turn_right'
        
        # If no obstacles, keep moving forward
        return 'forward' 