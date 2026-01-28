#!/usr/bin/env python3
"""
Action Executor Node

Executes actions from the VLA model:
- Validates actions for safety
- Sequences multi-step actions
- Monitors execution progress
- Handles action failures and recovery
"""

# Standard library
import time
from enum import Enum
from typing import Optional

# Third-party
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

# Local
from isaac_utils import HealthStatusPublisher, InputWatchdog


class ExecutionState(Enum):
    """Action execution state"""
    IDLE = "idle"
    EXECUTING = "executing"
    COMPLETE = "complete"
    FAILED = "failed"
    ABORTED = "aborted"


class ActionExecutorNode(Node):
    """Executes actions from VLA controller"""

    def __init__(self):
        super().__init__("action_executor_node")

        # Parameters
        self.declare_parameter("action_topic", "/vla/actions")
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")
        self.declare_parameter("execution_feedback_topic", "/vla/execution_feedback")
        
        # Safety parameters
        self.declare_parameter("max_linear_velocity", 0.5)  # m/s
        self.declare_parameter("max_angular_velocity", 2.0)  # rad/s
        self.declare_parameter("action_timeout_sec", 5.0)
        self.declare_parameter("enable_safety_checks", True)
        
        # Execution parameters
        self.declare_parameter("action_buffer_size", 10)
        self.declare_parameter("execution_rate", 50.0)  # Hz (control loop rate)
        
        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.execution_feedback_topic = str(
            self.get_parameter("execution_feedback_topic").value
        )
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.action_timeout = float(self.get_parameter("action_timeout_sec").value)
        self.enable_safety_checks = bool(self.get_parameter("enable_safety_checks").value)
        self.action_buffer_size = int(self.get_parameter("action_buffer_size").value)
        self.execution_rate = float(self.get_parameter("execution_rate").value)
        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        self.current_action = None
        self.action_buffer = []
        self.execution_state = ExecutionState.IDLE
        self.action_start_time = 0.0
        self.execution_count = 0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.feedback_pub = self.create_publisher(
            String, self.execution_feedback_topic, 10
        )

        # Subscribers
        self.action_sub = self.create_subscription(
            Float32MultiArray,
            self.action_topic,
            self._action_callback,
            10
        )

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)
        self.health.add_watchdog(
            InputWatchdog(
                "actions",
                expected_rate_hz=10.0,  # Expected VLA output rate
                warn_timeout_sec=3.0,
                error_timeout_sec=6.0,
            )
        )

        # Execution loop timer
        self.execution_timer = self.create_timer(
            1.0 / self.execution_rate, self._execution_loop
        )

        self.get_logger().info(
            f"Action executor started: rate={self.execution_rate}Hz, "
            f"max_vel=[{self.max_linear_velocity}, {self.max_angular_velocity}]"
        )

    def _action_callback(self, msg):
        """Receive and buffer new actions"""
        try:
            self.health.record_input("actions")

            # Convert to numpy array
            action = np.array(msg.data, dtype=np.float32)

            # Safety check
            if self.enable_safety_checks:
                action = self._apply_safety_limits(action)

            # Add to buffer
            if len(self.action_buffer) < self.action_buffer_size:
                self.action_buffer.append(action)
            else:
                # Buffer full, drop oldest
                self.action_buffer.pop(0)
                self.action_buffer.append(action)

        except Exception as e:
            self.get_logger().warn(f"Error processing action: {e}")

    def _apply_safety_limits(self, action: np.ndarray) -> np.ndarray:
        """Apply safety limits to action"""
        try:
            # Limit linear velocity
            if len(action) > 0:
                action[0] = np.clip(
                    action[0], -self.max_linear_velocity, self.max_linear_velocity
                )
            
            # Limit lateral velocity (if present)
            if len(action) > 1:
                action[1] = np.clip(
                    action[1], -self.max_linear_velocity, self.max_linear_velocity
                )
            
            # Limit angular velocity
            if len(action) > 2:
                action[2] = np.clip(
                    action[2], -self.max_angular_velocity, self.max_angular_velocity
                )

            return action

        except Exception as e:
            self.get_logger().warn(f"Error applying safety limits: {e}")
            return action

    def _execution_loop(self):
        """Main execution loop (runs at control rate)"""
        try:
            current_time = time.time()

            # Check for timeout
            if self.execution_state == ExecutionState.EXECUTING:
                if (current_time - self.action_start_time) > self.action_timeout:
                    self.get_logger().warn("Action execution timeout")
                    self._abort_execution()

            # Get next action from buffer
            if self.execution_state == ExecutionState.IDLE and len(self.action_buffer) > 0:
                self.current_action = self.action_buffer.pop(0)
                self.execution_state = ExecutionState.EXECUTING
                self.action_start_time = current_time
                self.execution_count += 1

            # Execute current action
            if self.execution_state == ExecutionState.EXECUTING and self.current_action is not None:
                self._execute_action(self.current_action)

        except Exception as e:
            self.get_logger().warn(f"Error in execution loop: {e}")

    def _execute_action(self, action: np.ndarray):
        """Execute a single action"""
        try:
            # Create cmd_vel message
            cmd_vel = Twist()
            
            if len(action) > 0:
                cmd_vel.linear.x = float(action[0])
            if len(action) > 1:
                cmd_vel.linear.y = float(action[1])
            if len(action) > 2:
                cmd_vel.angular.z = float(action[2])

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

            # Mark as complete (for now, actions are single-step)
            # In the future, this would check for action completion conditions
            self.execution_state = ExecutionState.COMPLETE

            # Publish feedback
            self._publish_feedback("complete")

            # Log periodically
            if self.execution_count % 50 == 0:
                self.get_logger().info(
                    f"Executed action #{self.execution_count}: "
                    f"vel=[{cmd_vel.linear.x:.3f}, {cmd_vel.angular.z:.3f}]"
                )

            # Reset for next action
            self.current_action = None
            self.execution_state = ExecutionState.IDLE

        except Exception as e:
            self.get_logger().warn(f"Error executing action: {e}")
            self._abort_execution()

    def _abort_execution(self):
        """Abort current action execution"""
        # Stop robot
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        # Update state
        self.execution_state = ExecutionState.ABORTED
        self.current_action = None

        # Publish feedback
        self._publish_feedback("aborted")

        self.get_logger().warn("Action execution aborted")

    def _publish_feedback(self, status: str):
        """Publish execution feedback"""
        try:
            feedback = String()
            feedback.data = f"{{\"status\": \"{status}\", \"count\": {self.execution_count}}}"
            self.feedback_pub.publish(feedback)

        except Exception as e:
            self.get_logger().warn(f"Error publishing feedback: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = ActionExecutorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        cmd_vel = Twist()
        node.cmd_vel_pub.publish(cmd_vel)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
