import math

class NavigationController:
    """Handles high-level navigation decisions."""
    
    def __init__(self, turn_speed=0.7, linear_speed=0.3):
        self.TURN_SPEED = turn_speed
        self.LINEAR_SPEED = linear_speed
        self.MIN_TURN_ANGLE = 80
        self.MAX_TURN_ANGLE = 100
        
    def calculate_turn_progress(self, current_angle, start_angle):
        """Calculate turn progress in degrees."""
        diff = math.degrees(current_angle - start_angle)
        
        angle_turned = diff % 360
        if angle_turned < 0:
            angle_turned += 360
            
        self.last_angle = angle_turned
        return angle_turned
        
    def is_turn_complete(self, angle_turned):
        """Check if turn is complete based on angle."""
        return self.MIN_TURN_ANGLE <= angle_turned <= self.MAX_TURN_ANGLE 