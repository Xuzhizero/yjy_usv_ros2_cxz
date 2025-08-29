# simple_py_node/hello_node.py

import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello from ROS 2 Python Node!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
