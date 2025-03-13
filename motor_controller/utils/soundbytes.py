#!/usr/bin/env python3
"""
Sound patterns for robot audio feedback.
Each function returns a pattern that can be played by the buzzer.

Format: [(frequency, volume, duration), ...]
"""

def startup_jingle():
    """Ascending chord progression for robot startup"""
    return [
        (523, 0.5, 0.1),    # C5
        (659, 0.5, 0.1),    # E5
        (784, 0.6, 0.1),    # G5
        (659, 0.6, 0.05),   # E5
        (784, 0.7, 0.05),   # G5
        (988, 0.7, 0.1),    # B5
        (784, 0.7, 0.05),   # G5
        (988, 0.8, 0.05),   # B5
        (1047, 0.8, 0.1),   # C6
        (1319, 0.9, 0.2),   # E6
        (1568, 1.0, 0.3)    # G6
    ]

def shutdown_jingle():
    """Descending scale for robot shutdown"""
    return [
        (1568, 0.6, 0.1),   # G6
        (1397, 0.6, 0.1),   # F6
        (1319, 0.5, 0.1),   # E6
        (1175, 0.5, 0.1),   # D6
        (1047, 0.4, 0.1),   # C6
        (988, 0.4, 0.1),    # B5
        (880, 0.3, 0.1),    # A5
        (784, 0.3, 0.2),    # G5
        (659, 0.2, 0.2),    # E5
        (523, 0.1, 0.3)     # C5
    ]

def human_detected_jingle():
    """Alert pattern with repeating motif for human detection"""
    return [
        (880, 0.7, 0.07)   # A5 - Alert
    ]

def escape_jingle():
    """Urgent pattern with rapid alternating tones for escape mode"""
    return [
        (1397, 0.8, 0.07),  # F6
        (1175, 0.8, 0.07),  # D6
        (1397, 0.85, 0.07), # F6
        (1175, 0.85, 0.07), # D6
        (1397, 0.9, 0.07),  # F6
        (1175, 0.9, 0.07),  # D6
        (1568, 0.95, 0.07), # G6
        (1397, 0.95, 0.07), # F6
        (1568, 1.0, 0.07),  # G6
        (1397, 1.0, 0.07),  # F6
        (1047, 1.0, 0.2)    # C6
    ]

def hiding_jingle():
    """Subtle descending pattern with decreasing volume for hiding"""
    return [
        (880, 0.5, 0.1),    # A5
        (784, 0.4, 0.1),    # G5
        (698, 0.3, 0.1),    # F5
        (659, 0.25, 0.1),   # E5
        (587, 0.2, 0.1),    # D5
        (523, 0.15, 0.1),   # C5
        (494, 0.1, 0.2),    # B4
        (440, 0.05, 0.3)    # A4
    ]

def success_jingle():
    """Happy triumphant pattern for success"""
    return [
        (784, 0.5, 0.1),    # G5
        (784, 0.6, 0.1),    # G5
        (1175, 0.7, 0.2),   # D6
        (0, 0, 0.05),       # Brief pause
        (1175, 0.7, 0.15),  # D6
        (1319, 0.8, 0.15),  # E6
        (1175, 0.8, 0.1),   # D6
        (1047, 0.8, 0.1),   # C6
        (988, 0.7, 0.1),    # B5
        (1047, 0.8, 0.1),   # C6
        (1175, 0.9, 0.1),   # D6
        (1568, 1.0, 0.3)    # G6
    ]

def error_jingle():
    """Error notification pattern with descending dissonance"""
    return [
        (1175, 0.8, 0.1),   # D6
        (1108, 0.85, 0.1),  # C#6 (dissonant with D6)
        (0, 0, 0.05),       # Brief pause
        (1175, 0.9, 0.1),   # D6 
        (1108, 0.95, 0.1),  # C#6
        (0, 0, 0.05),       # Brief pause
        (740, 0.9, 0.2),    # F#5
        (698, 0.85, 0.2),   # F5
        (622, 0.8, 0.3)     # D#5
    ]

def warning_jingle():
    """Warning notification pattern with repeating tones"""
    return [
        (880, 0.7, 0.1),    # A5
        (0, 0, 0.05),       # Pause
        (880, 0.8, 0.1),    # A5
        (0, 0, 0.05),       # Pause
        (880, 0.9, 0.1),    # A5
        (0, 0, 0.05),       # Pause
        (880, 1.0, 0.2),    # A5
        (784, 0.8, 0.1),    # G5
        (880, 1.0, 0.3)     # A5
    ]

def low_battery_jingle():
    """Low battery alert pattern with descending tones"""
    return [
        (587, 0.6, 0.1),    # D5
        (554, 0.6, 0.1),    # C#5
        (0, 0, 0.1),        # Longer pause
        (587, 0.7, 0.1),    # D5
        (554, 0.7, 0.1),    # C#5
        (0, 0, 0.1),        # Longer pause
        (466, 0.7, 0.1),    # A#4
        (440, 0.6, 0.1),    # A4
        (415, 0.5, 0.2),    # G#4
        (392, 0.4, 0.3)     # G4
    ]

def new_waypoint_jingle():
    """Bright ascending pattern for new waypoint"""
    return [
        (784, 0.5, 0.08),   # G5
        (880, 0.6, 0.08),   # A5
        (988, 0.6, 0.08),   # B5
        (1047, 0.7, 0.08),  # C6
        (1175, 0.7, 0.08),  # D6
        (1319, 0.8, 0.15),  # E6
        (1568, 0.9, 0.2)    # G6
    ]

def exploring_jingle():
    """Playful pattern for exploration mode"""
    return [
        (659, 0.5, 0.1),    # E5
        (784, 0.5, 0.1),    # G5
        (988, 0.6, 0.1),    # B5
        (784, 0.6, 0.1),    # G5
        (988, 0.7, 0.1),    # B5
        (1175, 0.7, 0.1),   # D6
        (988, 0.6, 0.1),    # B5
        (784, 0.5, 0.1)     # G5
    ]

def searching_jingle():
    """Curious pattern for searching behavior"""
    return [
        (523, 0.4, 0.1),    # C5
        (587, 0.5, 0.1),    # D5
        (659, 0.5, 0.1),    # E5
        (698, 0.6, 0.15),   # F5
        (0, 0, 0.05),       # Pause
        (659, 0.5, 0.1),    # E5
        (698, 0.6, 0.15),   # F5
        (784, 0.6, 0.2)     # G5
    ]

def stuck_jingle():
    """Frustrated pattern for when robot is stuck"""
    return [
        (523, 0.7, 0.1),    # C5
        (494, 0.7, 0.1),    # B4
        (466, 0.7, 0.1),    # A#4
        (0, 0, 0.05),       # Pause
        (523, 0.8, 0.1),    # C5
        (494, 0.8, 0.1),    # B4
        (466, 0.8, 0.15),   # A#4
        (0, 0, 0.05),       # Pause
        (523, 0.9, 0.15),   # C5
        (494, 0.9, 0.15),   # B4
        (440, 0.8, 0.3)     # A4
    ]

def thinking_jingle():
    """Contemplative pattern for processing/thinking state"""
    return [
        (659, 0.4, 0.1),    # E5
        (784, 0.5, 0.1),    # G5
        (880, 0.5, 0.15),   # A5
        (0, 0, 0.3),        # Longer pause
        (659, 0.4, 0.1),    # E5
        (784, 0.5, 0.1),    # G5
        (880, 0.5, 0.15),   # A5
        (0, 0, 0.3),        # Longer pause
        (988, 0.6, 0.3)     # B5
    ]

def rotate_jingle():
    """Swirling pattern for rotation actions"""
    return [
        (523, 0.5, 0.08),   # C5
        (587, 0.5, 0.08),   # D5
        (659, 0.5, 0.08),   # E5
        (698, 0.5, 0.08),   # F5
        (784, 0.6, 0.08),   # G5
        (880, 0.6, 0.08),   # A5
        (988, 0.6, 0.08),   # B5
        (1047, 0.6, 0.08),  # C6
        (988, 0.6, 0.08),   # B5
        (880, 0.5, 0.08),   # A5
        (784, 0.5, 0.08),   # G5
        (698, 0.5, 0.08),   # F5
        (659, 0.4, 0.08),   # E5
        (587, 0.4, 0.08),   # D5
        (523, 0.4, 0.08)    # C5
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
    "new_waypoint": new_waypoint_jingle,
    "exploring": exploring_jingle,
    "searching": searching_jingle,
    "stuck": stuck_jingle,
    "thinking": thinking_jingle,
    "rotate": rotate_jingle
}

def get_jingle(name):
    """Get a jingle pattern by name"""
    if name in jingle_map:
        return jingle_map[name]()
    else:
        # Default beep if jingle not found
        return [(1000, 0.7, 0.2)]

