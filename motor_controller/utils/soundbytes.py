#!/usr/bin/env python3
"""
Sound patterns for robot audio feedback.
Each function returns a pattern that can be played by the buzzer.

Format: [(frequency, volume, duration), ...]
"""

def startup_jingle():
    """Ascending tone sequence for robot startup"""
    return [
        (784, 0.5, 0.1),    # G5
        (988, 0.5, 0.1),    # B5
        (1319, 0.7, 0.15),  # E6
        (1568, 0.8, 0.3)    # G6
    ]

def shutdown_jingle():
    """Descending tone sequence for robot shutdown"""
    return [
        (1568, 0.5, 0.1),   # G6
        (1319, 0.5, 0.1),   # E6
        (988, 0.5, 0.1),    # B5
        (784, 0.7, 0.3)     # G5
    ]

def human_detected_jingle():
    """Alert pattern for human detection"""
    return [
        (1047, 0.6, 0.1),   # C6
        (1047, 0.7, 0.1),   # C6
        (1175, 0.7, 0.1),   # D6
        (1319, 0.8, 0.2)    # E6
    ]

def escape_jingle():
    """Urgent pattern for escape mode"""
    return [
        (1397, 0.8, 0.1),   # F6
        (1175, 0.8, 0.1),   # D6
        (1397, 0.9, 0.1),   # F6
        (1175, 0.9, 0.1),   # D6
        (1047, 1.0, 0.2)    # C6
    ]

def hiding_jingle():
    """Subtle pattern for hiding"""
    return [
        (880, 0.4, 0.1),    # A5
        (698, 0.3, 0.2),    # F5
        (587, 0.2, 0.3)     # D5
    ]

def success_jingle():
    """Happy pattern for success"""
    return [
        (784, 0.5, 0.1),    # G5
        (988, 0.6, 0.1),    # B5
        (1175, 0.7, 0.1),   # D6
        (1568, 0.8, 0.3)    # G6
    ]

def error_jingle():
    """Error notification pattern"""
    return [
        (1175, 0.7, 0.1),   # D6
        (622, 0.8, 0.3),    # D#5
        (622, 0.9, 0.3)     # D#5
    ]

def warning_jingle():
    """Warning notification pattern"""
    return [
        (880, 0.7, 0.1),    # A5
        (880, 0.8, 0.1),    # A5
        (880, 0.9, 0.3)     # A5
    ]

def low_battery_jingle():
    """Low battery alert pattern"""
    return [
        (466, 0.6, 0.1),    # A#4
        (466, 0.7, 0.1),    # A#4
        (415, 0.8, 0.3)     # G#4
    ]

def new_waypoint_jingle():
    """Short confirmation for new waypoint"""
    return [
        (784, 0.5, 0.1),    # G5
        (988, 0.5, 0.1),    # B5
        (1319, 0.7, 0.15),  # E6
        (1568, 0.8, 0.3)    # G6
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
    "new_waypoint": new_waypoint_jingle
}

def get_jingle(name):
    """Get a jingle pattern by name"""
    if name in jingle_map:
        return jingle_map[name]()
    else:
        # Default beep if jingle not found
        return [(1000, 0.5, 0.2)]

