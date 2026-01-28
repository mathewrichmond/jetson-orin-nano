#!/usr/bin/env python3
"""
Audio Pipeline Orchestrator Node

Manages the audio processing pipeline:
- Coordinates feature extraction and speech recognition
- Health monitoring for audio subsystems  
- Power-aware operation (reduce processing in low battery)
- Module health aggregation
"""

# Standard library
import time
from typing import Dict, Optional

# Third-party
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Local
from custom_msgs.msg import ModuleHealth, NodeHealth, PowerRequest
from isaac_utils import HealthStatusPublisher


class AudioPipelineNode(Node):
    """Audio pipeline orchestrator and health monitor"""

    def __init__(self):
        super().__init__("audio_pipeline_node")

        # Parameters
        self.declare_parameter("output_namespace", "/jetson")
        self.declare_parameter("audio_namespace", "/audio")
        
        # Audio processing configuration
        self.declare_parameter("initial_mode", "FULL")  # FULL, REDUCED, SLEEP
        self.declare_parameter("enable_feature_extraction", True)
        self.declare_parameter("enable_speech_recognition", True)
        
        # Power management integration
        self.declare_parameter("auto_mode_switching", True)
        self.declare_parameter("battery_threshold_reduced", 25.0)  # %
        self.declare_parameter("battery_threshold_sleep", 15.0)  # %
        self.declare_parameter("temp_threshold_reduce", 75.0)  # °C
        
        # Health monitoring
        self.declare_parameter("health_publish_rate", 1.0)
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("module_health_topic", "/jetson/health/audio_pipeline")

        # Get parameters
        self.output_namespace = str(self.get_parameter("output_namespace").value)
        self.audio_namespace = str(self.get_parameter("audio_namespace").value)
        self.mode = str(self.get_parameter("initial_mode").value)
        self.enable_feature_extraction = bool(
            self.get_parameter("enable_feature_extraction").value
        )
        self.enable_speech_recognition = bool(
            self.get_parameter("enable_speech_recognition").value
        )
        self.auto_mode_switching = bool(self.get_parameter("auto_mode_switching").value)
        self.battery_threshold_reduced = float(
            self.get_parameter("battery_threshold_reduced").value
        )
        self.battery_threshold_sleep = float(
            self.get_parameter("battery_threshold_sleep").value
        )
        self.temp_threshold_reduce = float(self.get_parameter("temp_threshold_reduce").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)
        health_topic = str(self.get_parameter("health_topic").value)
        module_health_topic = str(self.get_parameter("module_health_topic").value)

        # State
        self.battery_level = 100.0
        self.cpu_temp = 50.0
        self.last_audio_time = 0.0
        self.node_health: Dict[str, dict] = {}

        # Publishers
        self.module_health_pub = self.create_publisher(
            ModuleHealth, module_health_topic, 10
        )
        self.power_request_pub = self.create_publisher(
            PowerRequest, f"{self.output_namespace}/power/request", 10
        )

        # Subscribers
        self.battery_sub = self.create_subscription(
            # BatteryState type - using String for now (will be fixed with proper import)
            String,
            "/irobot/battery",
            self._battery_callback,
            10
        )
        self.temp_sub = self.create_subscription(
            # Temperature type - using String for now
            String,
            f"{self.output_namespace}/system/temperature/cpu",
            self._temp_callback,
            10
        )

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)

        # Module health publication timer
        self.module_health_timer = self.create_timer(
            1.0 / health_rate, self._publish_module_health
        )

        self.get_logger().info(
            f"Audio pipeline orchestrator started: mode={self.mode}, "
            f"feature_extraction={self.enable_feature_extraction}, "
            f"speech_recognition={self.enable_speech_recognition}"
        )

    def _battery_callback(self, msg):
        """Monitor battery level"""
        try:
            # TODO: Parse BatteryState message when proper import is available
            # For now, assume battery level is in msg.data
            # self.battery_level = msg.percentage * 100
            
            # Placeholder: extract from string representation
            if hasattr(msg, 'data'):
                # Try to parse battery percentage from string
                pass
            
            # Auto mode switching based on battery
            if self.auto_mode_switching:
                self._check_mode_switching()
                
        except Exception as e:
            self.get_logger().warn(f"Error processing battery update: {e}")

    def _temp_callback(self, msg):
        """Monitor CPU temperature"""
        try:
            # TODO: Parse Temperature message when proper import is available
            # For now, assume temperature is in msg.data
            # self.cpu_temp = msg.temperature
            
            # Placeholder
            if hasattr(msg, 'data'):
                pass
            
            # Auto mode switching based on temperature
            if self.auto_mode_switching:
                self._check_mode_switching()
                
        except Exception as e:
            self.get_logger().warn(f"Error processing temperature update: {e}")

    def _check_mode_switching(self):
        """Check if mode should be switched based on battery/temperature"""
        new_mode = None
        
        # Battery-based switching
        if self.battery_level < self.battery_threshold_sleep:
            new_mode = "SLEEP"
        elif self.battery_level < self.battery_threshold_reduced:
            new_mode = "REDUCED"
        else:
            new_mode = "FULL"
        
        # Temperature-based override
        if self.cpu_temp > self.temp_threshold_reduce and new_mode == "FULL":
            new_mode = "REDUCED"
        
        # Switch mode if changed
        if new_mode != self.mode:
            self._switch_mode(new_mode)

    def _switch_mode(self, new_mode: str):
        """Switch audio processing mode"""
        old_mode = self.mode
        self.mode = new_mode
        
        self.get_logger().info(f"Audio mode switched: {old_mode} -> {new_mode}")
        
        # Publish power request based on mode
        self._publish_power_request()

    def _publish_power_request(self):
        """Publish power request based on current mode"""
        try:
            request = PowerRequest()
            request.requester = self.get_name()
            request.timestamp = self.get_clock().now().to_msg()
            request.priority = 2  # Medium priority (0-4 scale)
            request.timeout_sec = 30.0
            request.reason = f"Audio pipeline mode: {self.mode}"
            
            if self.mode == "FULL":
                request.requested_state = "ON"
            elif self.mode == "REDUCED":
                request.requested_state = "ECONOMY"
            else:  # SLEEP
                request.requested_state = "SLEEP"
            
            self.power_request_pub.publish(request)
            
        except Exception as e:
            self.get_logger().warn(f"Error publishing power request: {e}")

    def _publish_module_health(self):
        """Publish aggregated module health"""
        try:
            module_health = ModuleHealth()
            module_health.module_name = "audio_pipeline"
            module_health.host_name = "jetson"  # TODO: Get actual hostname
            module_health.timestamp = self.get_clock().now().to_msg()
            
            # Aggregate node health (placeholder - would track actual nodes)
            nodes = []
            
            # Feature extractor health
            if self.enable_feature_extraction:
                node = NodeHealth()
                node.node_name = "audio_feature_extractor"
                node.status = "HEALTHY"  # Would be tracked from actual node
                nodes.append(node)
            
            # Speech recognition health
            if self.enable_speech_recognition:
                node = NodeHealth()
                node.node_name = "speech_recognition"
                node.status = "HEALTHY"  # Would be tracked from actual node
                nodes.append(node)
            
            # Orchestrator health
            node = NodeHealth()
            node.node_name = self.get_name()
            node.status = "HEALTHY"
            nodes.append(node)
            
            module_health.nodes = nodes
            
            # Overall status (worst of all nodes)
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
    node = AudioPipelineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
