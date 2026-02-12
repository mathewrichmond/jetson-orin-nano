#!/usr/bin/env python3
"""
Jetson Control Adapter

Bridges platform mission commands to ROS 2 VLA controller and subsystem commands.
"""

# Standard library
import threading
from typing import Any, Dict, Optional

# Third-party
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robotics_common import TimeSync

# Platform SDK
from robotics_sdk import ControlBase
from std_msgs.msg import String


class JetsonControl(ControlBase):
    """
    Jetson control adapter.

    Bridges platform mission commands to ROS 2 VLA controller:
    - Mission commands → VLA controller
    - VLA execution feedback → Mission status
    - Subsystem commands → Hardware controllers
    """

    def __init__(self, robot_id: str, ros2_node: Node, zenoh_bridge=None):
        """
        Initialize Jetson control adapter.

        Args:
            robot_id: Unique robot identifier
            ros2_node: ROS 2 node for publications/subscriptions
            zenoh_bridge: ZenohBridge instance (if None, creates new session)
        """
        super().__init__(robot_id, zenoh_bridge)

        self.ros2_node = ros2_node
        self.time_sync = TimeSync(use_ptp=True)

        # Current mission tracking
        self.current_mission: Optional[Dict[str, Any]] = None
        self.mission_lock = threading.Lock()

        # ROS 2 publishers
        self.vla_command_pub = self.ros2_node.create_publisher(String, "/vla/command", 10)

        self.cmd_vel_pub = self.ros2_node.create_publisher(Twist, "/control/cmd_vel", 10)

        # ROS 2 subscribers
        self._setup_subscriptions()

        # Subscribe to platform mission commands
        self.subscribe_missions(self.on_mission)

        self.ros2_node.get_logger().info(f"[JetsonControl] Initialized for robot {robot_id}")

    def _setup_subscriptions(self):
        """Set up ROS 2 subscriptions to monitor execution."""

        # VLA execution feedback
        self.ros2_node.create_subscription(
            String, "/vla/execution_feedback", self._vla_feedback_callback, 10
        )

        # VLA plan status
        self.ros2_node.create_subscription(
            String, "/vla/plan_status", self._vla_plan_status_callback, 10
        )

        self.ros2_node.get_logger().info("[JetsonControl] Subscriptions created")

    def on_mission(self, mission: Dict[str, Any]):
        """
        Handle incoming mission command from platform.

        Args:
            mission: Mission dictionary with keys:
                - mission_id: Unique mission identifier
                - command: Natural language command
                - required_capabilities: List of required capabilities
                - priority: Mission priority (1-5)
                - timeout: Timeout in seconds
                - parameters: Additional parameters (JSON string)
        """
        try:
            with self.mission_lock:
                self.current_mission = mission

            mission_id = mission.get("mission_id", "unknown")
            command = mission.get("command", "")

            self.ros2_node.get_logger().info(
                f"[JetsonControl] Received mission {mission_id}: {command}"
            )

            # Publish status: in_progress
            self.publish_mission_status(mission_id, "in_progress")

            # Forward command to VLA controller
            vla_msg = String()
            vla_msg.data = command
            self.vla_command_pub.publish(vla_msg)

            self.ros2_node.get_logger().info(f"[JetsonControl] Forwarded to VLA: {command}")

        except Exception as e:
            self.ros2_node.get_logger().error(f"Error handling mission: {e}")
            if self.current_mission:
                mission_id = self.current_mission.get("mission_id", "unknown")
                self.publish_mission_result(mission_id, "failed", f"Error: {str(e)}")

    def _vla_feedback_callback(self, msg: String):
        """
        Handle VLA execution feedback.

        VLA publishes execution updates like "Moving cameras to target"
        or "Action complete".
        """
        feedback = msg.data

        with self.mission_lock:
            if self.current_mission is None:
                return

            mission_id = self.current_mission.get("mission_id", "unknown")

        # Check if execution complete
        if "complete" in feedback.lower() or "done" in feedback.lower():
            self.ros2_node.get_logger().info(f"[JetsonControl] Mission {mission_id} completed")
            self.publish_mission_result(mission_id, "completed", feedback)

            with self.mission_lock:
                self.current_mission = None

        elif "error" in feedback.lower() or "failed" in feedback.lower():
            self.ros2_node.get_logger().error(
                f"[JetsonControl] Mission {mission_id} failed: {feedback}"
            )
            self.publish_mission_result(mission_id, "failed", feedback)

            with self.mission_lock:
                self.current_mission = None

        else:
            # Progress update
            self.publish_mission_status(mission_id, "in_progress", feedback)

    def _vla_plan_status_callback(self, msg: String):
        """
        Handle VLA planning status updates.

        VLA publishes planning status like "Planning action sequence"
        or "Executing action 2/5".
        """
        status = msg.data

        with self.mission_lock:
            if self.current_mission is None:
                return

            mission_id = self.current_mission.get("mission_id", "unknown")

        # Forward planning status as mission status update
        self.publish_mission_status(mission_id, "in_progress", status)

    def send_base_velocity(self, linear_x: float, angular_z: float):
        """
        Send velocity command to mobile base.

        Args:
            linear_x: Linear velocity (m/s)
            angular_z: Angular velocity (rad/s)
        """
        try:
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.cmd_vel_pub.publish(cmd)

        except Exception as e:
            self.ros2_node.get_logger().error(f"Error sending base velocity: {e}")

    def send_pantilt_goal(self, camera_id: str, pan: float, tilt: float):
        """
        Send pan-tilt goal for a camera.

        Args:
            camera_id: Camera identifier ("left" or "right")
            pan: Pan angle in degrees
            tilt: Tilt angle in degrees
        """
        # TODO: Implement pan-tilt goal publishing
        # This requires interfacing with the pan-tilt controller
        # or publishing to appropriate ROS 2 topics

        self.ros2_node.get_logger().info(
            f"[JetsonControl] Pan-tilt goal for {camera_id}: " f"pan={pan}°, tilt={tilt}°"
        )

    def start(self):
        """Start control processing."""
        super().start()
        self.ros2_node.get_logger().info("[JetsonControl] Started")

    def stop(self):
        """Stop control processing."""
        super().stop()
        self.ros2_node.get_logger().info("[JetsonControl] Stopped")
