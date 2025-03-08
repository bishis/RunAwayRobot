#!/usr/bin/env python3
import enum
from typing import Optional, Callable, Dict, Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import time


class NavigationState(enum.Enum):
    """Possible states for the navigation controller."""
    INITIALIZING = "initializing"  # Waiting for Nav2 to be ready
    IDLE = "idle"  # Not actively navigating
    EXPLORING = "exploring"  # Autonomous exploration
    HUMAN_TRACKING = "human_tracking"  # Tracking and reacting to a human
    ESCAPING = "escaping"  # Executing an escape maneuver
    SHAKE_DEFENSE = "shake_defense"  # Performing shake defense when trapped
    POST_ESCAPE = "post_escape"  # After successful escape, turning to face human
    ERROR = "error"  # Error state


class NavigationEvent(enum.Enum):
    """Events that can trigger state transitions."""
    NAV2_READY = "nav2_ready"  # Nav2 stack is ready
    EXPLORATION_REQUESTED = "exploration_requested"  # Start exploration
    HUMAN_DETECTED = "human_detected"  # Human detected
    HUMAN_LOST = "human_lost"  # Human no longer detected
    ESCAPE_NEEDED = "escape_needed"  # Critical human distance, need to escape
    ESCAPE_SUCCEEDED = "escape_succeeded"  # Escape plan completed successfully
    ESCAPE_FAILED = "escape_failed"  # Escape plan failed
    TRAPPED = "trapped"  # Robot is trapped, can't find escape path
    GOAL_REACHED = "goal_reached"  # Navigation goal reached
    GOAL_FAILED = "goal_failed"  # Navigation goal failed
    GOAL_TIMEOUT = "goal_timeout"  # Navigation timeout
    STUCK = "stuck"  # Robot is stuck
    RESUME = "resume"  # Resume normal operation
    ERROR_OCCURRED = "error_occurred"  # An error occurred
    ERROR_RESOLVED = "error_resolved"  # Error has been resolved


class NavigationFSM:
    """Finite State Machine for robot navigation control."""

    def __init__(self, node: Node, callbacks: Dict[str, Callable]):
        """
        Initialize the Navigation FSM.
        
        Args:
            node: The ROS node this FSM is attached to
            callbacks: Dictionary of callback functions for each state and transition
        """
        self.node = node
        self.current_state = NavigationState.INITIALIZING
        self.previous_state = None
        self.callbacks = callbacks
        self.state_entry_time = self.node.get_clock().now()
        self.state_data = {}  # Store state-specific data

        # Define state transition table
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
                NavigationEvent.GOAL_REACHED: NavigationState.EXPLORING,  # Continue exploring
                NavigationEvent.GOAL_FAILED: NavigationState.EXPLORING,  # Retry exploring
                NavigationEvent.GOAL_TIMEOUT: NavigationState.EXPLORING,  # Generate new waypoint
                NavigationEvent.STUCK: NavigationState.EXPLORING,  # Attempt recovery
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.HUMAN_TRACKING: {
                NavigationEvent.HUMAN_LOST: NavigationState.IDLE,
                NavigationEvent.ESCAPE_NEEDED: NavigationState.ESCAPING,
                NavigationEvent.ERROR_OCCURRED: NavigationState.ERROR
            },
            NavigationState.ESCAPING: {
                NavigationEvent.ESCAPE_SUCCEEDED: NavigationState.POST_ESCAPE,
                NavigationEvent.ESCAPE_FAILED: NavigationState.ESCAPING,  # Retry escape
                NavigationEvent.TRAPPED: NavigationState.SHAKE_DEFENSE,
                NavigationEvent.GOAL_TIMEOUT: NavigationState.ESCAPING,  # Retry with new plan
                NavigationEvent.STUCK: NavigationState.ESCAPING,  # Retry with new plan
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

        # Initialize the state
        self.log_state_transition(None, self.current_state)
        self.execute_state_callback("on_enter")
        
    def trigger_event(self, event: NavigationEvent, data: Any = None) -> bool:
        """
        Trigger an event to potentially change the state.
        
        Args:
            event: The event that occurred
            data: Optional data to pass to the callback functions
            
        Returns:
            True if state changed, False otherwise
        """
        if event not in NavigationEvent:
            self.node.get_logger().error(f"Invalid event: {event}")
            return False
            
        # Check if transition is valid for current state
        if self.current_state not in self.transitions or event not in self.transitions[self.current_state]:
            self.node.get_logger().warn(
                f"No transition defined for event {event} in state {self.current_state}"
            )
            return False
            
        # Get new state from transition table
        new_state = self.transitions[self.current_state][event]
        
        # If no state change, just return
        if new_state == self.current_state:
            return False
            
        # Execute exit callback for current state
        self.execute_state_callback("on_exit", event=event, data=data)
        
        # Update state
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = self.node.get_clock().now()
        self.state_data = data if data is not None else {}
        
        # Log state transition
        self.log_state_transition(event, new_state)
        
        # Execute entry callback for new state
        self.execute_state_callback("on_enter", event=event, data=data)
        
        return True
        
    def execute_state_callback(self, callback_type: str, event: Optional[NavigationEvent] = None, data: Any = None):
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
                self.node.get_logger().error(
                    f"Error executing callback {callback_name}: {str(e)}"
                )
        
        # Also execute generic callbacks if they exist
        generic_callback = f"{callback_type}_any"
        if generic_callback in self.callbacks:
            try:
                self.callbacks[generic_callback](
                    state=self.current_state,
                    event=event,
                    data=data
                )
            except Exception as e:
                self.node.get_logger().error(
                    f"Error executing generic callback {generic_callback}: {str(e)}"
                )
    
    def update(self, data: Any = None):
        """
        Update the current state. This should be called regularly.
        
        Args:
            data: Optional data to pass to the update callback
        """
        # Execute update callback for current state
        self.execute_state_callback("on_update", data=data)
        
    def get_state_duration(self) -> float:
        """
        Get the duration that the FSM has been in the current state (in seconds).
        
        Returns:
            Duration in seconds
        """
        current_time = self.node.get_clock().now()
        return (current_time - self.state_entry_time).nanoseconds / 1e9
        
    def is_in_state(self, state: NavigationState) -> bool:
        """
        Check if the FSM is in the specified state.
        
        Args:
            state: The state to check
            
        Returns:
            True if in the specified state, False otherwise
        """
        return self.current_state == state
        
    def log_state_transition(self, event: Optional[NavigationEvent], new_state: NavigationState):
        """
        Log a state transition.
        
        Args:
            event: The event that triggered the transition
            new_state: The new state
        """
        if event:
            self.node.get_logger().info(
                f"State transition: {self.current_state} -> {new_state} (triggered by {event})"
            )
        else:
            self.node.get_logger().info(
                f"Initial state: {new_state}"
            )
            
    def get_valid_events(self) -> list:
        """
        Get a list of events that are valid from the current state.
        
        Returns:
            List of valid events
        """
        if self.current_state in self.transitions:
            return list(self.transitions[self.current_state].keys())
        return []
        
    def force_state(self, state: NavigationState, data: Any = None):
        """
        Force the FSM into a specific state, bypassing normal transitions.
        Use with caution!
        
        Args:
            state: The state to force
            data: Optional data to store with the state
        """
        # Log the forced transition
        self.node.get_logger().warn(
            f"Forcing state transition: {self.current_state} -> {state}"
        )
        
        # Execute exit callback for current state
        self.execute_state_callback("on_exit")
        
        # Update state
        self.previous_state = self.current_state
        self.current_state = state
        self.state_entry_time = self.node.get_clock().now()
        self.state_data = data if data is not None else {}
        
        # Execute entry callback for new state
        self.execute_state_callback("on_enter")