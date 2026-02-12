#!/usr/bin/env python3
"""
Jetson Robot Implementation

Main entry point for platform integration. Wraps perception and control adapters.
"""

# Standard library
import os
from typing import Optional

# Third-party
import rclpy
from rclpy.node import Node

# Platform SDK
from robotics_sdk import RobotBase
import yaml

from .jetson_control import JetsonControl
from .jetson_perception import JetsonPerception


class JetsonRobot(RobotBase):
    """
    Jetson robot implementation.

    Integrates Jetson hardware with the cross-platform robotics system.
    Manages perception and control adapters.
    """

    def __init__(
        self,
        robot_id: Optional[str] = None,
        config_path: Optional[str] = None,
        zenoh_router: Optional[str] = None,
    ):
        """
        Initialize Jetson robot.

        Args:
            robot_id: Unique robot identifier (defaults to $HOST_ID or "jetson-01")
            config_path: Path to capability.yaml (defaults to package config)
            zenoh_router: Zenoh router address or None for auto-discovery
        """
        # Get robot ID from environment or default
        if robot_id is None:
            robot_id = os.environ.get("HOST_ID", "jetson-01")

        # Load capability configuration
        if config_path is None:
            # Default to package config directory
            package_dir = os.path.dirname(os.path.dirname(__file__))
            config_path = os.path.join(package_dir, "config", "capability.yaml")

        with open(config_path, "r") as f:
            self.capability_config = yaml.safe_load(f)

        # Extract capabilities from config
        capabilities = self.capability_config.get("capabilities", [])

        # Initialize base with Zenoh session
        super().__init__(robot_id, capabilities, zenoh_router)

        # Initialize ROS 2
        rclpy.init()
        self.ros2_node = Node("platform_bridge")

        # Initialize perception and control adapters
        self.perception = JetsonPerception(robot_id, self.ros2_node, self.zenoh_bridge)

        self.control = JetsonControl(robot_id, self.ros2_node, self.zenoh_bridge)

        self.ros2_node.get_logger().info(f"[JetsonRobot] Initialized robot {robot_id}")

    def register_with_platform(self):
        """
        Register robot capabilities with the platform dispatch service.

        Publishes capability descriptor to /capabilities/robot/{robot_id}
        """
        sensors = self.capability_config.get("sensors", {})
        actuators = self.capability_config.get("actuators", {})

        self.register_capabilities(sensors=sensors, actuators=actuators)

        self.ros2_node.get_logger().info("[JetsonRobot] Registered capabilities with platform")

    def start(self):
        """Start robot operation."""
        super().start()

        # Start perception and control
        self.perception.start()
        self.control.start()

        # Register with platform
        self.register_with_platform()

        self.ros2_node.get_logger().info("[JetsonRobot] Robot started")

    def stop(self):
        """Stop robot operation."""
        # Stop adapters
        self.perception.stop()
        self.control.stop()

        # Shutdown ROS 2
        self.ros2_node.destroy_node()
        rclpy.shutdown()

        # Stop base
        super().stop()

        print("[JetsonRobot] Robot stopped")

    def spin(self):
        """
        Spin ROS 2 node to process callbacks.

        This should be called in the main loop to keep the robot running.
        """
        try:
            rclpy.spin(self.ros2_node)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main():
    """Main entry point for Jetson robot platform bridge."""
    robot = JetsonRobot()

    try:
        robot.start()
        robot.spin()
    except KeyboardInterrupt:
        print("\n[JetsonRobot] Shutting down...")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
