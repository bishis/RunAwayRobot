class RobotState:
    """Manages robot pose and motion state."""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Frame names
        self.base_frame = 'base_footprint'  # Changed to base_footprint
        self.odom_frame = 'odom'
        self.map_frame = 'map'
        self.laser_frame = 'laser'

    def update_pose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def update_velocity(self, linear, angular):
        self.linear_vel = linear
        self.angular_vel = angular 