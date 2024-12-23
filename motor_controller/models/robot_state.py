class RobotState:
    """Manages robot pose and motion state."""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.is_turning = False
        self.start_turn_angle = 0.0
        self.target_angle = 0.0

    def update_pose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def update_velocity(self, linear, angular):
        self.linear_vel = linear
        self.angular_vel = angular 