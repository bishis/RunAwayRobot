#!/usr/bin/env python3
"""
Sound patterns for robot audio feedback.
Each function returns a pattern that can be played by the buzzer.
"""

def startup_jingle():
    """Ascending tone sequence for robot startup"""
    return [
        (784, 0.1, 0.05),  # G5
        (988, 0.1, 0.05),  # B5
        (1319, 0.15, 0.05), # E6
        (1568, 0.3, 0)     # G6
    ]

def shutdown_jingle():
    """Descending tone sequence for robot shutdown"""
    return [
        (1568, 0.1, 0.05),  # G6
        (1319, 0.1, 0.05),  # E6
        (988, 0.1, 0.05),   # B5
        (784, 0.3, 0)       # G5
    ]

def human_detected_jingle():
    """Alert pattern for human detection"""
    return [
        (1047, 0.1, 0.05),  # C6
        (1047, 0.1, 0.05),  # C6
        (1175, 0.1, 0.05),  # D6
        (1319, 0.2, 0)      # E6
    ]

def escape_jingle():
    """Urgent pattern for escape mode"""
    return [
        (1397, 0.1, 0.05),  # F6
        (1175, 0.1, 0.05),  # D6
        (1397, 0.1, 0.05),  # F6
        (1175, 0.1, 0.05),  # D6
        (1047, 0.2, 0)      # C6
    ]

def hiding_jingle():
    """Subtle pattern for hiding"""
    return [
        (880, 0.1, 0.05),   # A5
        (698, 0.2, 0.05),   # F5
        (587, 0.3, 0)       # D5
    ]

def success_jingle():
    """Happy pattern for success"""
    return [
        (784, 0.1, 0.05),   # G5
        (988, 0.1, 0.05),   # B5
        (1175, 0.1, 0.05),  # D6
        (1568, 0.3, 0)      # G6
    ]

def error_jingle():
    """Error notification pattern"""
    return [
        (1175, 0.1, 0.05),  # D6
        (622, 0.3, 0.05),   # D#5
        (622, 0.3, 0)       # D#5
    ]

def warning_jingle():
    """Warning notification pattern"""
    return [
        (880, 0.1, 0.05),   # A5
        (880, 0.1, 0.1),    # A5
        (880, 0.3, 0)       # A5
    ]

def low_battery_jingle():
    """Low battery alert pattern"""
    return [
        (466, 0.1, 0.05),   # A#4
        (466, 0.1, 0.2),    # A#4
        (415, 0.3, 0)       # G#4
    ]

def follow_me_jingle():
    """Friendly pattern when following human"""
    return [
        (784, 0.1, 0.05),   # G5
        (880, 0.1, 0.05),   # A5
        (988, 0.1, 0.05),   # B5
        (1047, 0.2, 0)      # C6
    ]

def obstacle_detected_jingle():
    """Short alert for obstacle detection"""
    return [
        (622, 0.05, 0.05),  # D#5
        (622, 0.05, 0)      # D#5
    ]

def new_waypoint_jingle():
    """Short confirmation for new waypoint"""
    return [
        (1047, 0.05, 0.05), # C6
        (1319, 0.1, 0)      # E6
    ]

# Map jingle names to functions for easy lookup
jingle_map = {
    "startup": startup_jingle,
    "shutdown": shutdown_jingle,
    "human_detected": human_detected_jingle,
    "escape": escape_jingle,
    "hiding": hiding_jingle,
    "success": success_jingle,
    "error": error_jingle,
    "warning": warning_jingle,
    "low_battery": low_battery_jingle,
    "follow_me": follow_me_jingle,
    "obstacle": obstacle_detected_jingle,
    "new_waypoint": new_waypoint_jingle
}

def get_jingle(name):
    """Get a jingle pattern by name"""
    if name in jingle_map:
        return jingle_map[name]()
    else:
        # Default beep if jingle not found
        return [(1000, 0.2, 0)]

