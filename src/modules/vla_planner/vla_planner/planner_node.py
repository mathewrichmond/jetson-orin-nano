#!/usr/bin/env python3
"""
Planner Node

High-level task planning and coordination:
- Manages task queue and priorities
- Coordinates VLA controller and action executor
- Handles task success/failure
- Provides planning feedback
"""

# Standard library
import time
from collections import deque
from enum import Enum
from typing import Dict, List, Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Local
from custom_msgs.msg import ModuleHealth, NodeHealth
from isaac_utils import HealthStatusPublisher


class TaskState(Enum):
    """Task execution state"""
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETE = "complete"
    FAILED = "failed"
    CANCELLED = "cancelled"


class Task:
    """Task representation"""
    def __init__(self, task_id: str, description: str, priority: int = 0):
        self.task_id = task_id
        self.description = description
        self.priority = priority
        self.state = TaskState.PENDING
        self.start_time = 0.0
        self.end_time = 0.0
        self.retries = 0


class PlannerNode(Node):
    """High-level planner and task coordinator"""

    def __init__(self):
        super().__init__("planner_node")

        # Parameters
        self.declare_parameter("task_queue_size", 10)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("task_timeout_sec", 60.0)
        self.declare_parameter("planning_rate", 1.0)  # Hz
        
        # Topics
        self.declare_parameter("command_topic", "/vla/command")
        self.declare_parameter("transcription_topic", "/audio/transcription")
        self.declare_parameter("execution_feedback_topic", "/vla/execution_feedback")
        self.declare_parameter("plan_status_topic", "/vla/plan_status")
        
        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("module_health_topic", "/jetson/health/vla_planner")

        # Get parameters
        self.task_queue_size = int(self.get_parameter("task_queue_size").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.task_timeout = float(self.get_parameter("task_timeout_sec").value)
        self.planning_rate = float(self.get_parameter("planning_rate").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.transcription_topic = str(self.get_parameter("transcription_topic").value)
        self.execution_feedback_topic = str(
            self.get_parameter("execution_feedback_topic").value
        )
        self.plan_status_topic = str(self.get_parameter("plan_status_topic").value)
        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        module_health_topic = str(self.get_parameter("module_health_topic").value)

        # State
        self.task_queue: deque = deque(maxlen=self.task_queue_size)
        self.current_task: Optional[Task] = None
        self.completed_tasks: List[Task] = []
        self.task_counter = 0

        # Publishers
        self.plan_status_pub = self.create_publisher(
            String, self.plan_status_topic, 10
        )
        self.module_health_pub = self.create_publisher(
            ModuleHealth, module_health_topic, 10
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._command_callback,
            10
        )
        self.transcription_sub = self.create_subscription(
            String,
            self.transcription_topic,
            self._transcription_callback,
            10
        )
        self.feedback_sub = self.create_subscription(
            String,
            self.execution_feedback_topic,
            self._feedback_callback,
            10
        )

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)

        # Planning loop timer
        self.planning_timer = self.create_timer(
            1.0 / self.planning_rate, self._planning_loop
        )

        # Module health timer
        self.module_health_timer = self.create_timer(
            1.0 / health_rate, self._publish_module_health
        )

        self.get_logger().info(
            f"Planner started: queue_size={self.task_queue_size}, "
            f"planning_rate={self.planning_rate}Hz"
        )

    def _command_callback(self, msg):
        """Receive explicit task commands"""
        try:
            command = msg.data.strip()
            
            if command:
                self._add_task(command, priority=5)  # High priority for explicit commands
                
        except Exception as e:
            self.get_logger().warn(f"Error processing command: {e}")

    def _transcription_callback(self, msg):
        """Parse transcription for task commands"""
        try:
            transcription = msg.data.strip()
            
            # Simple command parsing (placeholder)
            if transcription and len(transcription) > 0:
                # Look for action keywords
                keywords = ["go", "move", "turn", "stop", "pick", "place"]
                if any(keyword in transcription.lower() for keyword in keywords):
                    self._add_task(transcription, priority=3)  # Medium priority
                    
        except Exception as e:
            self.get_logger().warn(f"Error parsing transcription: {e}")

    def _feedback_callback(self, msg):
        """Receive execution feedback"""
        try:
            feedback = msg.data
            
            # Check if current task completed
            if self.current_task is not None and "complete" in feedback:
                self._complete_current_task()
            elif self.current_task is not None and "aborted" in feedback:
                self._fail_current_task()
                
        except Exception as e:
            self.get_logger().warn(f"Error processing feedback: {e}")

    def _add_task(self, description: str, priority: int = 0):
        """Add new task to queue"""
        try:
            self.task_counter += 1
            task = Task(
                task_id=f"task_{self.task_counter}",
                description=description,
                priority=priority
            )
            
            # Add to queue
            self.task_queue.append(task)
            
            self.get_logger().info(
                f"Added task: {task.task_id} - '{description}' (priority={priority})"
            )
            
            # Publish status update
            self._publish_status()
            
        except Exception as e:
            self.get_logger().warn(f"Error adding task: {e}")

    def _planning_loop(self):
        """Main planning loop"""
        try:
            current_time = time.time()

            # Check for task timeout
            if self.current_task is not None:
                if (current_time - self.current_task.start_time) > self.task_timeout:
                    self.get_logger().warn(f"Task timeout: {self.current_task.task_id}")
                    self._fail_current_task()

            # Get next task if idle
            if self.current_task is None and len(self.task_queue) > 0:
                # Sort queue by priority (highest first)
                sorted_queue = sorted(
                    self.task_queue, key=lambda t: t.priority, reverse=True
                )
                
                # Get highest priority task
                next_task = sorted_queue[0]
                self.task_queue.remove(next_task)
                
                # Start task
                self._start_task(next_task)

        except Exception as e:
            self.get_logger().warn(f"Error in planning loop: {e}")

    def _start_task(self, task: Task):
        """Start executing a task"""
        try:
            task.state = TaskState.ACTIVE
            task.start_time = time.time()
            self.current_task = task
            
            self.get_logger().info(f"Starting task: {task.task_id} - '{task.description}'")
            
            # Publish status
            self._publish_status()
            
            # In a real implementation, this would:
            # 1. Break down task into subtasks
            # 2. Generate plan with VLA controller
            # 3. Monitor execution progress
            
        except Exception as e:
            self.get_logger().warn(f"Error starting task: {e}")
            self._fail_current_task()

    def _complete_current_task(self):
        """Mark current task as complete"""
        if self.current_task is None:
            return
            
        self.current_task.state = TaskState.COMPLETE
        self.current_task.end_time = time.time()
        
        duration = self.current_task.end_time - self.current_task.start_time
        
        self.get_logger().info(
            f"Completed task: {self.current_task.task_id} "
            f"(duration={duration:.1f}s)"
        )
        
        self.completed_tasks.append(self.current_task)
        self.current_task = None
        
        # Publish status
        self._publish_status()

    def _fail_current_task(self):
        """Mark current task as failed and retry if possible"""
        if self.current_task is None:
            return
            
        self.current_task.retries += 1
        
        if self.current_task.retries < self.max_retries:
            # Retry
            self.get_logger().warn(
                f"Retrying task: {self.current_task.task_id} "
                f"(attempt {self.current_task.retries + 1}/{self.max_retries})"
            )
            self.current_task.state = TaskState.PENDING
            self.task_queue.append(self.current_task)
        else:
            # Max retries reached
            self.get_logger().error(
                f"Task failed: {self.current_task.task_id} (max retries reached)"
            )
            self.current_task.state = TaskState.FAILED
            self.current_task.end_time = time.time()
            self.completed_tasks.append(self.current_task)
        
        self.current_task = None
        
        # Publish status
        self._publish_status()

    def _publish_status(self):
        """Publish planning status"""
        try:
            status = {
                "current_task": self.current_task.task_id if self.current_task else None,
                "queue_size": len(self.task_queue),
                "completed": len(self.completed_tasks)
            }
            
            status_msg = String()
            status_msg.data = str(status)
            self.plan_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error publishing status: {e}")

    def _publish_module_health(self):
        """Publish aggregated module health"""
        try:
            module_health = ModuleHealth()
            module_health.module_name = "vla_planner"
            module_health.host_name = "jetson"  # TODO: Get actual hostname
            module_health.timestamp = self.get_clock().now().to_msg()
            
            # Create node health entries
            nodes = []
            
            # VLA controller health
            node = NodeHealth()
            node.node_name = "vla_controller"
            node.status = "HEALTHY"  # Would be tracked from actual node
            nodes.append(node)
            
            # Action executor health
            node = NodeHealth()
            node.node_name = "action_executor"
            node.status = "HEALTHY"  # Would be tracked from actual node
            nodes.append(node)
            
            # Planner health
            node = NodeHealth()
            node.node_name = self.get_name()
            node.status = "HEALTHY"
            nodes.append(node)
            
            module_health.nodes = nodes
            
            # Overall status
            statuses = [n.status for n in nodes]
            if "ERROR" in statuses:
                module_health.overall_status = "ERROR"
            elif "WARN" in statuses:
                module_health.overall_status = "WARN"
            else:
                module_health.overall_status = "HEALTHY"
            
            self.module_health_pub.publish(module_health)
            
        except Exception as e:
            self.get_logger().warn(f"Error publishing module health: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = PlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
