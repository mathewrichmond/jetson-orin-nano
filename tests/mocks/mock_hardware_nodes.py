"""
Mock Hardware Nodes for Testing

Provides mock implementations of hardware nodes for testing without real devices.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

from tests.mocks.mock_sensor_data import MockIMUData, MockCameraData


class MockIMUNode(Node):
    """Mock IMU node for testing"""
    
    def __init__(self, node_name: str = "mock_imu"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("topic", "/hardware/phat/imu")
        
        publish_rate = self.get_parameter("publish_rate").value
        topic = self.get_parameter("topic").value
        
        # Mock data generator
        self.mock_imu = MockIMUData(frequency=publish_rate)
        
        # Publisher
        self.publisher = self.create_publisher(Imu, topic, 10)
        
        # Timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Mock IMU node started, publishing to {topic} at {publish_rate} Hz")
    
    def timer_callback(self):
        """Publish mock IMU data"""
        data = self.mock_imu.generate()
        
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        msg.linear_acceleration.x = data["linear_acceleration"]["x"]
        msg.linear_acceleration.y = data["linear_acceleration"]["y"]
        msg.linear_acceleration.z = data["linear_acceleration"]["z"]
        
        msg.angular_velocity.x = data["angular_velocity"]["x"]
        msg.angular_velocity.y = data["angular_velocity"]["y"]
        msg.angular_velocity.z = data["angular_velocity"]["z"]
        
        self.publisher.publish(msg)


class MockCameraNode(Node):
    """Mock camera node for testing"""
    
    def __init__(self, node_name: str = "mock_camera"):
        super().__init__(node_name)
        
        # Parameters
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("topic", "/camera/color/image_raw")
        
        fps = self.get_parameter("fps").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        topic = self.get_parameter("topic").value
        
        # Mock data generator
        self.mock_camera = MockCameraData(width=width, height=height, fps=fps)
        
        # Publisher
        self.publisher = self.create_publisher(Image, topic, 10)
        
        # Timer
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Mock camera node started, publishing to {topic} at {fps} FPS")
    
    def timer_callback(self):
        """Publish mock camera frame"""
        frame = self.mock_camera.generate()
        
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        
        self.publisher.publish(msg)


def main_mock_imu(args=None):
    """Main entry point for mock IMU node"""
    rclpy.init(args=args)
    node = MockIMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_mock_camera(args=None):
    """Main entry point for mock camera node"""
    rclpy.init(args=args)
    node = MockCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
