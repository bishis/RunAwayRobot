#!/usr/bin/env python3
import enum
from typing import Optional, Callable, Dict, Any
import rclpy
from rclpy.node import Node
import math

class NavigationState(enum.Enum):
    """Possible states for the navigation controller."""
    INITIALIZING = "initializing"      # Waiting for Nav2 stack to be ready
    IDLE = "idle"                      # Nothing active, waiting for command
    EXPLORING = "exploring"            # Autonomous exploration
    HUMAN_TRACKING = "human_tracking"  # Reacting to human presence
    ESCAPING = "escaping"              # Executing an escape maneuver
    SHAKE_DEFENSE = "shake_defense"    # Shaking to free from being trapped
    POST_ESCAPE = "post_escape"        # After escape: turning to face human
    ERROR = "error"                    # Error state

class NavigationEvent(enum.Enum):
    """Events that can trigger state transitions."""
    NAV2_READY = "nav2_ready"              # Nav2 stack is ready
    EXPLORATION_REQUESTED = "exploration_requested"  # Begin exploration
    HUMAN_DETECTED = "human_detected"      # Human detected
    HUMAN_LOST = "human_lost"              # Human no longer detected
    ESCAPE_NEEDED = "escape_needed"        # Critical distance to human; must escape
    ESCAPE_SUCCEEDED = "escape_succeeded"  # Escape maneuver succeeded
    ESCAPE_FAILED = "escape_failed"        # Escape maneuver failed
    TRAPPED = "trapped"                    # Robot stuck during goal execution
    GOAL_REACHED = "goal_reached"          # Goal reached successfully
    GOAL_FAILED = "goal_failed"            # Goal failed
    GOAL_TIMEOUT = "goal_timeout"          # Goal took too long
    STUCK = "stuck"                        # Robot hasn't moved sufficiently
    RESUME = "resume"                      # Resume normal behavior
    ERROR_OCCURRED = "error_occurred"      # An error occurred
    ERROR_RESOLVED = "error_resolved"      # Error resolved

class NavigationFSM:
    """Simple FSM engine for navigation control."""
    def __init__(self, node: Node, callbacks: dict):
        self.node = node
        self.current_state = NavigationState.INITIALIZING
        self.previous_state = None
        self.callbacks = callbacks
        self.state_entry_time = self.node.get_clock().now()
        self.state_data = {}
        # Define allowed transitions:
        self.transitions = {
            NavigationState.INITIALIZING: {
                NavigationEvent.NAV2_READY: NavigationState.IDLE,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.IDLE: {
                NavigationEvent.EXPLORATION_REQUESTED: NavigationState.EXPLORING,
                NavigationEvent.HUMAN_DETECTED: NavigationState.HUMAN_TRACKING,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.EXPLORING: {
                NavigationEvent.HUMAN_DETECTED: NavigationState.HUMAN_TRACKING,
                NavigationEvent.HUMAN_LOST: NavigationState.EXPLORING,  # Just continue exploring if human lost
                NavigationEvent.GOAL_REACHED: NavigationState.EXPLORING,  # continue exploring
                NavigationEvent.GOAL_FAILED: NavigationState.EXPLORING,   # retry exploration
                NavigationEvent.GOAL_TIMEOUT: NavigationState.EXPLORING,    # generate new waypoint
                NavigationEvent.STUCK: NavigationState.EXPLORING,           # recover
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.HUMAN_TRACKING: {
                NavigationEvent.HUMAN_LOST: NavigationState.IDLE,
                NavigationEvent.ESCAPE_NEEDED: NavigationState.ESCAPING,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.ESCAPING: {
                NavigationEvent.ESCAPE_SUCCEEDED: NavigationState.POST_ESCAPE,
                NavigationEvent.ESCAPE_FAILED: NavigationState.ESCAPING,  # retry escape
                NavigationEvent.TRAPPED: NavigationState.SHAKE_DEFENSE,
                NavigationEvent.GOAL_TIMEOUT: NavigationState.ESCAPING,
                NavigationEvent.STUCK: NavigationState.ESCAPING,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.POST_ESCAPE: {
                NavigationEvent.HUMAN_DETECTED: NavigationState.HUMAN_TRACKING,
                NavigationEvent.RESUME: NavigationState.IDLE,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.SHAKE_DEFENSE: {
                NavigationEvent.HUMAN_LOST: NavigationState.IDLE,
                NavigationEvent.RESUME: NavigationState.IDLE,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.ERROR: {
                NavigationEvent.ERROR_RESOLVED: NavigationState.IDLE
            }
        }
        self.log_state_transition(None, self.current_state)
        self.execute_state_callback("on_enter")
    
    def trigger_event(self, event: NavigationEvent, data: any = None) -> bool:
        """
        Trigger an event to potentially change the state.
        
        Args:
            event: The event that occurred
            data: Optional data to pass to the callback functions
            
        Returns:
            True if state changed, False otherwise
        """
        if self.current_state not in self.transitions or event not in self.transitions[self.current_state]:
            self.node.get_logger().warn(f"No transition for event {event} in state {self.current_state}")
            return False
            
        new_state = self.transitions[self.current_state][event]
        
        if new_state == self.current_state:
            return False
            
        # Execute exit callback for current state
        self.execute_state_callback("on_exit", event, data)
        
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = self.node.get_clock().now()
        self.state_data = data if data is not None else {}
        
        self.log_state_transition(event, new_state)
        
        # Execute entry callback for new state
        self.execute_state_callback("on_enter", event, data)
        
        return True

    def execute_state_callback(self, callback_type: str, event: NavigationEvent = None, data: any = None):
        """
        Execute a callback for the current state.
        
        Args:
            callback_type: Type of callback (on_enter, on_exit, on_update)
            event: The event that triggered this callback, if any
            data: Optional data to pass to the callback
        """
        callback_name = f"{callback_type}_{self.current_state.value}"
        
        if callback_name in self.callbacks:
            try:
                self.callbacks[callback_name](event=event, data=data)
            except Exception as e:
                self.node.get_logger().error(f"Error in {callback_name}: {e}")
                
        generic_callback = f"{callback_type}_any"
        if generic_callback in self.callbacks:
            try:
                self.callbacks[generic_callback](state=self.current_state, event=event, data=data)
            except Exception as e:
                self.node.get_logger().error(f"Error in {generic_callback}: {e}")

    def update(self, data: any = None):
        """
        Update the current state. This should be called regularly.
        
        Args:
            data: Optional data to pass to the update callback
        """
        self.execute_state_callback("on_update", data=data)

    def get_state_duration(self) -> float:
        """
        Get the duration that the FSM has been in the current state (in seconds).
        
        Returns:
            Duration in seconds
        """
        current_time = self.node.get_clock().now()
        return (current_time - self.state_entry_time).nanoseconds / 1e9

    def log_state_transition(self, event: NavigationEvent, new_state: NavigationState):
        """
        Log a state transition.
        
        Args:
            event: The event that triggered the transition
            new_state: The new state
        """
        if event:
            self.node.get_logger().info(
                f"State transition: {self.previous_state} -> {new_state} (triggered by {event})"
            )
        else:
            self.node.get_logger().info(f"Initial state: {new_state}")

    def force_state(self, state: NavigationState, data: any = None):
        """
        Force the FSM into a specific state, bypassing normal transitions.
        Use with caution!
        
        Args:
            state: The state to force
            data: Optional data to store with the state
        """
        self.node.get_logger().warn(f"Forcing state transition: {self.current_state} -> {state}")
        self.execute_state_callback("on_exit")
        self.previous_state = self.current_state
        self.current_state = state
        self.state_entry_time = self.node.get_clock().now()
        self.state_data = data if data is not None else {}
        self.execute_state_callback("on_enter")