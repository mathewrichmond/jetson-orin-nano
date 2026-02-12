#!/usr/bin/env python3
"""
VLA Controller Node

Runs VLA (Vision-Language-Action) model inference:
- Consumes multimodal sensor data (vision, audio, proprioception)
- Runs VLA model inference
- Outputs action sequences
- Handles model loading and GPU management
"""

# Standard library
import time
from typing import Optional

# Third-party
# Local
from custom_msgs.msg import PowerRequest
from geometry_msgs.msg import Twist
from isaac_utils import HealthStatusPublisher, InputWatchdog
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class VLAControllerNode(Node):
    """VLA model inference controller"""

    def __init__(self):
        super().__init__("vla_controller_node")

        # Parameters
        self.declare_parameter("model_path", "")  # Path to VLA model weights
        self.declare_parameter("model_type", "placeholder")  # placeholder, openvla, rt1, etc.
        self.declare_parameter("device", "cuda")  # cuda, cpu
        self.declare_parameter("inference_rate", 10.0)  # Hz

        # Input topic parameters
        self.declare_parameter("vision_features_topic", "/sensor_fusion/vlm_features")
        self.declare_parameter("audio_features_topic", "/audio/features/mfcc")
        self.declare_parameter("robot_state_topic", "/rpi/chassis/pose_estimate")
        self.declare_parameter("transcription_topic", "/audio/transcription")

        # Output topic parameters
        self.declare_parameter("action_topic", "/vla/actions")
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")

        # Model parameters
        self.declare_parameter("action_horizon", 10)  # Number of future actions to predict
        self.declare_parameter("context_window", 5)  # Number of past observations to use
        self.declare_parameter("confidence_threshold", 0.5)  # Min confidence to execute action

        # Power management
        self.declare_parameter("request_gpu", True)
        self.declare_parameter("gpu_priority", 4)  # Highest priority

        # Health monitoring
        self.declare_parameter("health_topic", f"health/{self.get_name()}")
        self.declare_parameter("health_publish_rate", 1.0)

        # Get parameters
        self.model_path = str(self.get_parameter("model_path").value)
        self.model_type = str(self.get_parameter("model_type").value)
        self.device = str(self.get_parameter("device").value)
        self.inference_rate = float(self.get_parameter("inference_rate").value)
        self.vision_features_topic = str(self.get_parameter("vision_features_topic").value)
        self.audio_features_topic = str(self.get_parameter("audio_features_topic").value)
        self.robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        self.transcription_topic = str(self.get_parameter("transcription_topic").value)
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.action_horizon = int(self.get_parameter("action_horizon").value)
        self.context_window = int(self.get_parameter("context_window").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.request_gpu = bool(self.get_parameter("request_gpu").value)
        self.gpu_priority = int(self.get_parameter("gpu_priority").value)
        health_topic = str(self.get_parameter("health_topic").value)
        health_rate = float(self.get_parameter("health_publish_rate").value)

        # State
        self.model = None
        self.model_loaded = False
        self.last_vision_features = None
        self.last_audio_features = None
        self.last_robot_state = None
        self.last_transcription = ""
        self.inference_count = 0

        # Publishers
        self.action_pub = self.create_publisher(Float32MultiArray, self.action_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.power_request_pub = self.create_publisher(PowerRequest, "/jetson/power/request", 10)

        # Subscribers
        self.vision_sub = self.create_subscription(
            String,  # TODO: Replace with actual VLM features message type
            self.vision_features_topic,
            self._vision_callback,
            10,
        )
        self.audio_sub = self.create_subscription(
            Float32MultiArray, self.audio_features_topic, self._audio_callback, 10
        )
        self.robot_state_sub = self.create_subscription(
            # TODO: Replace with actual Pose message type
            String,
            self.robot_state_topic,
            self._robot_state_callback,
            10,
        )
        self.transcription_sub = self.create_subscription(
            String, self.transcription_topic, self._transcription_callback, 10
        )

        # Platform mission command subscription
        self.command_sub = self.create_subscription(
            String, "/vla/command", self._command_callback, 10
        )

        # Current mission command
        self.current_command: Optional[str] = None
        self.command_timestamp = 0.0

        # Execution feedback publisher
        self.execution_feedback_pub = self.create_publisher(String, "/vla/execution_feedback", 10)
        self.plan_status_pub = self.create_publisher(String, "/vla/plan_status", 10)

        # Health monitoring
        self.health = HealthStatusPublisher(self, health_topic, health_rate)
        self.health.add_watchdog(
            InputWatchdog(
                "vision_features",
                expected_rate_hz=self.inference_rate,
                warn_timeout_sec=3.0,
                error_timeout_sec=6.0,
            )
        )

        # Inference timer
        self.inference_timer = self.create_timer(
            1.0 / self.inference_rate, self._inference_callback
        )

        # Load model
        self._load_model()

        # Request GPU power
        if self.request_gpu:
            self._request_gpu_power()

        self.get_logger().info(
            f"VLA controller started: model_type={self.model_type}, "
            f"device={self.device}, rate={self.inference_rate}Hz"
        )

    def _load_model(self):
        """Load VLA model (placeholder)"""
        try:
            if self.model_type == "placeholder":
                self.get_logger().info("Using placeholder VLA model (no actual inference)")
                self.model = None
                self.model_loaded = True
            elif self.model_type == "openvla":
                self.get_logger().warn("OpenVLA not yet implemented")
                # Future: Load OpenVLA model
                # from openvla import OpenVLA
                # self.model = OpenVLA.load(self.model_path, device=self.device)
                self.model_loaded = False
            elif self.model_type == "rt1":
                self.get_logger().warn("RT-1 not yet implemented")
                # Future: Load RT-1 model
                self.model_loaded = False
            else:
                self.get_logger().error(f"Unknown model type: {self.model_type}")
                self.model_loaded = False

        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            self.model_loaded = False

    def _request_gpu_power(self):
        """Request GPU power from power manager"""
        try:
            request = PowerRequest()
            request.requester = self.get_name()
            request.timestamp = self.get_clock().now().to_msg()
            request.requested_state = "ON"
            request.priority = self.gpu_priority
            request.timeout_sec = 0.0  # Indefinite
            request.reason = "VLA model inference"

            self.power_request_pub.publish(request)
            self.get_logger().info("Requested GPU power for VLA inference")

        except Exception as e:
            self.get_logger().warn(f"Error requesting GPU power: {e}")

    def _vision_callback(self, msg):
        """Store latest vision features"""
        self.health.record_input("vision_features")
        self.last_vision_features = msg

    def _audio_callback(self, msg):
        """Store latest audio features"""
        self.last_audio_features = msg

    def _robot_state_callback(self, msg):
        """Store latest robot state"""
        self.last_robot_state = msg

    def _transcription_callback(self, msg):
        """Store latest speech transcription"""
        self.last_transcription = msg.data

    def _command_callback(self, msg):
        """
        Handle incoming mission command from platform.

        Args:
            msg: String message containing natural language command
        """
        command = msg.data
        self.current_command = command
        self.command_timestamp = time.time()

        self.get_logger().info(f"[VLA] Received command: {command}")

        # Publish planning status
        status_msg = String()
        status_msg.data = f"Planning action sequence for: {command}"
        self.plan_status_pub.publish(status_msg)

        # In a real implementation, this would:
        # 1. Parse the natural language command
        # 2. Query the digital twin for context
        # 3. Generate action sequence
        # 4. Begin execution

        # For now, publish mock feedback
        feedback_msg = String()
        feedback_msg.data = f"Processing command: {command}"
        self.execution_feedback_pub.publish(feedback_msg)

        # After some processing, publish completion
        # (In real implementation, this would happen after actual execution)
        self.create_timer(
            2.0, lambda: self._publish_completion(command), one_shot=True  # Mock 2-second execution
        )

    def _publish_completion(self, command: str):
        """Publish mission completion feedback."""
        feedback_msg = String()
        feedback_msg.data = f"Mission complete: {command}"
        self.execution_feedback_pub.publish(feedback_msg)

        self.get_logger().info(f"[VLA] Completed command: {command}")
        self.current_command = None

    def _inference_callback(self):
        """Run VLA model inference"""
        try:
            # Check if we have all required inputs
            if self.last_vision_features is None:
                return

            if not self.model_loaded:
                return

            # Run inference
            actions = self._run_inference()

            # Publish actions
            if actions is not None:
                self._publish_actions(actions)
                self.inference_count += 1

        except Exception as e:
            self.get_logger().warn(f"Error in inference: {e}")

    def _run_inference(self) -> Optional[np.ndarray]:
        """
        Run VLA model inference (placeholder implementation)

        Real implementation would:
        1. Prepare multimodal input (vision, audio, language, proprioception)
        2. Run model forward pass
        3. Decode actions from model output
        4. Return action sequence
        """
        try:
            if self.model_type == "placeholder":
                # Mock inference: return simple forward motion
                # Action format: [linear_x, linear_y, angular_z, gripper, ...]
                actions = np.array([0.1, 0.0, 0.0, 0.0], dtype=np.float32)

                # Add some variation based on transcription (demo)
                if "stop" in self.last_transcription.lower():
                    actions[0] = 0.0
                elif "turn" in self.last_transcription.lower():
                    actions[2] = 0.5

                return actions
            else:
                # Real model inference would go here
                return None

        except Exception as e:
            self.get_logger().warn(f"Error running inference: {e}")
            return None

    def _publish_actions(self, actions: np.ndarray):
        """Publish action sequence"""
        try:
            # Publish raw actions
            action_msg = Float32MultiArray()
            action_msg.data = actions.tolist()
            self.action_pub.publish(action_msg)

            # Publish as cmd_vel (for mobile base control)
            if len(actions) >= 3:
                cmd_vel = Twist()
                cmd_vel.linear.x = float(actions[0])
                cmd_vel.linear.y = float(actions[1])
                cmd_vel.angular.z = float(actions[2])
                self.cmd_vel_pub.publish(cmd_vel)

            if self.inference_count % 100 == 0:
                self.get_logger().info(
                    f"VLA inference #{self.inference_count}: "
                    f"action=[{actions[0]:.3f}, {actions[1]:.3f}, {actions[2]:.3f}]"
                )

        except Exception as e:
            self.get_logger().warn(f"Error publishing actions: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = VLAControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
