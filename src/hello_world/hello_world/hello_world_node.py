#!/usr/bin/env python3
"""
Hello World Node
Simple ROS 2 node to verify the Isaac robot system is working
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class HelloWorldNode(Node):
    """Simple hello world ROS 2 node"""

    def __init__(self):
        super().__init__('hello_world')

        # Publisher
        self.publisher = self.create_publisher(String, 'hello_world/message', 10)

        # Timer to publish messages
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Counter
        self.counter = 0

        self.get_logger().info('Hello World Node started!')
        self.get_logger().info('Isaac robot system is operational!')

    def timer_callback(self):
        """Timer callback to publish messages"""
        msg = String()
        msg.data = f'Hello from Isaac Robot! Message #{self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
