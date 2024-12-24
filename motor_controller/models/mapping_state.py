from enum import Enum

class MappingState(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4

class MapperState:
    def __init__(self):
        self.current_state = MappingState.FORWARD
        self.segment_length = 1.0  # 1 meter segments
        self.turn_angle = 90.0     # 90 degree turns
        self.current_distance = 0.0
        self.current_angle = 0.0
        self.direction = 1  # 1 for right turn next, -1 for left turn next 