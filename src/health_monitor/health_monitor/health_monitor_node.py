#!/usr/bin/env python3
"""
Health Monitor Node
Aggregates per-node health topics into a system-level health summary.
"""

# Standard library
import json
from pathlib import Path
import time
from typing import Any, Dict, Optional

# Third-party
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class HealthMonitorNode(Node):
    """Aggregates node health status into system-level health."""

    def __init__(self):
        super().__init__("health_monitor_node")

        # Parameters
        self.declare_parameter("graph_config", "modular_graph.yaml")
        self.declare_parameter("group", "all")
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("health_warn_timeout_sec", 5.0)
        self.declare_parameter("health_stale_timeout_sec", 10.0)

        self.graph_config = str(self.get_parameter("graph_config").value)
        self.group = str(self.get_parameter("group").value)
        self.publish_rate = float(self.get_parameter("health_publish_rate").value)
        self.default_warn_timeout = float(self.get_parameter("health_warn_timeout_sec").value)
        self.default_stale_timeout = float(self.get_parameter("health_stale_timeout_sec").value)

        self.node_configs: Dict[str, Dict[str, Any]] = {}
        self.health_topics: Dict[str, str] = {}
        self.health_timeouts: Dict[str, Dict[str, float]] = {}
        self.last_health: Dict[str, Dict[str, Any]] = {}
        self.last_seen_time: Dict[str, float] = {}

        self._load_graph_config()
        self._create_subscribers()

        self.summary_pub = self.create_publisher(String, "/system/health/summary", 10)
        self.nodes_pub = self.create_publisher(String, "/system/health/nodes", 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_summary)
        self.get_logger().info(
            f"Health monitor started with {len(self.health_topics)} node(s) (group={self.group})"
        )

    def _resolve_config_path(self, config_file: str) -> Path:
        config_path = Path(config_file)
        if config_path.exists():
            return config_path

        # Try package share for isaac_robot
        try:
            share_dir = Path(get_package_share_directory("isaac_robot"))
            candidate = share_dir / "config" / "robot" / config_path.name
            if candidate.exists():
                return candidate
        except Exception:
            pass

        # Try centralized config directory
        for base in [Path("/home/nano/src/jetson-orin-nano"), Path("/opt/isaac-robot")]:
            candidate = base / "config" / "robot" / config_path.name
            if candidate.exists():
                return candidate

        # Try source space (legacy)
        candidate = Path(__file__).parent.parent.parent / "config" / "robot" / config_path.name
        if candidate.exists():
            return candidate

        raise FileNotFoundError(f"Graph config file not found: {config_file}")

    def _load_graph_config(self) -> None:
        config_path = self._resolve_config_path(self.graph_config)
        with open(config_path, "r") as file:
            config = yaml.safe_load(file) or {}

        nodes = config.get("robot", {})
        if self.group and self.group != "all":
            groups = config.get("groups", {})
            node_names = groups.get(self.group, [])
        else:
            node_names = list(nodes.keys())

        for node_name in node_names:
            node_config = nodes.get(node_name, {})
            if not node_config.get("enabled", True):
                continue
            self.node_configs[node_name] = node_config

        for node_name, node_config in self.node_configs.items():
            health_topic = self._health_topic_for_node(node_name, node_config)
            self.health_topics[node_name] = health_topic
            self.health_timeouts[node_name] = self._timeouts_for_node(node_config)

    def _health_topic_for_node(self, node_name: str, node_config: Dict[str, Any]) -> str:
        params = node_config.get("parameters", {})
        explicit_topic = params.get("health_topic")
        if explicit_topic:
            return explicit_topic

        namespace = str(node_config.get("namespace", "") or "")
        if namespace and not namespace.startswith("/"):
            namespace = f"/{namespace}"
        if namespace == "/":
            namespace = ""

        return f"{namespace}/health/{node_name}"

    def _timeouts_for_node(self, node_config: Dict[str, Any]) -> Dict[str, float]:
        params = node_config.get("parameters", {})
        publish_rate = float(params.get("health_publish_rate", self.publish_rate))
        warn_timeout = float(params.get("health_warn_timeout_sec", self.default_warn_timeout))
        stale_timeout = float(params.get("health_stale_timeout_sec", self.default_stale_timeout))
        if publish_rate > 0:
            stale_timeout = max(stale_timeout, 3.0 / publish_rate)
            warn_timeout = max(warn_timeout, 1.5 / publish_rate)
        return {"warn": warn_timeout, "stale": stale_timeout}

    def _create_subscribers(self) -> None:
        for node_name, topic in self.health_topics.items():
            self.create_subscription(
                String,
                topic,
                self._make_health_callback(node_name),
                10,
            )

    def _make_health_callback(self, node_name: str):
        def _callback(msg: String) -> None:
            try:
                data = json.loads(msg.data)
                self.last_health[node_name] = data
                self.last_seen_time[node_name] = time.time()
            except json.JSONDecodeError:
                self.last_health[node_name] = {"status": "ERROR", "errors": ["invalid_json"]}
                self.last_seen_time[node_name] = time.time()
        return _callback

    def _publish_summary(self) -> None:
        now = time.time()
        summary = {
            "timestamp": now,
            "overall_status": "OK",
            "nodes": {},
            "error_nodes": [],
            "warn_nodes": [],
            "stale_nodes": [],
        }

        severity = {"OK": 0, "INIT": 1, "WARN": 2, "ERROR": 3, "STALE": 3}
        worst = "OK"

        for node_name, topic in self.health_topics.items():
            last_seen = self.last_seen_time.get(node_name)
            age = None if last_seen is None else max(0.0, now - last_seen)
            timeouts = self.health_timeouts.get(node_name, {})
            warn_timeout = timeouts.get("warn", self.default_warn_timeout)
            stale_timeout = timeouts.get("stale", self.default_stale_timeout)

            if last_seen is None or (age is not None and age >= stale_timeout):
                status = "STALE"
            elif age is not None and age >= warn_timeout:
                status = "WARN"
            else:
                status = (self.last_health.get(node_name, {}).get("status") or "INIT").upper()

            node_entry = {
                "status": status,
                "last_seen_age_sec": age,
                "health_topic": topic,
            }
            summary["nodes"][node_name] = node_entry

            if status == "ERROR":
                summary["error_nodes"].append(node_name)
            elif status == "WARN":
                summary["warn_nodes"].append(node_name)
            elif status == "STALE":
                summary["stale_nodes"].append(node_name)

            if severity[status] > severity[worst]:
                worst = status

        summary["overall_status"] = worst

        summary_msg = String()
        summary_msg.data = json.dumps(summary)
        self.summary_pub.publish(summary_msg)

        nodes_msg = String()
        nodes_msg.data = json.dumps(self.last_health)
        self.nodes_pub.publish(nodes_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
