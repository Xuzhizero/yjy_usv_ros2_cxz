#!/usr/bin/env python3
# coding: utf-8
"""
Image Subscriber Node for ROS2
Subscribe to Raw Image stream from Simulink and process it.

This module provides:
- ImageSubscriber class: handles image subscription and processing
- ImageSubscriberNode: ROS2 node wrapper for the subscriber

Usage:
    ros2 run control_planner image_subscriber
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ImageSubscriber:
    """
    Image subscriber class that handles ROS2 image subscription.

    This class encapsulates the image subscription logic and provides
    callback handling for incoming image messages from Simulink.

    Attributes:
        node: ROS2 Node object for creating subscriptions
        bridge: CvBridge object for converting ROS Image to OpenCV format
        latest_image: The most recently received image (as numpy array)
        image_count: Counter for received images
        topic_name: The ROS2 topic name for image subscription
    """

    def __init__(self, node: Node, topic_name: str = "/simulink/image_raw"):
        """
        Initialize the ImageSubscriber.

        Args:
            node: ROS2 Node object
            topic_name: Topic name to subscribe to (default: /simulink/image_raw)
        """
        self.node = node
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_count = 0
        self.topic_name = topic_name

        # Create subscription to image topic
        self.subscription = node.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10  # QoS depth
        )

        node.get_logger().info(f"ImageSubscriber initialized, subscribing to: {self.topic_name}")

    def image_callback(self, msg: Image):
        """
        Callback function for incoming image messages.

        Args:
            msg: ROS2 Image message from Simulink
        """
        try:
            # Convert ROS Image message to OpenCV format (numpy array)
            # Assuming the image is in RGB8 or BGR8 format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_image = cv_image
            self.image_count += 1

            # Log image info periodically (every 100 frames)
            if self.image_count % 100 == 0:
                self.node.get_logger().info(
                    f"Received image #{self.image_count}, "
                    f"size: {cv_image.shape}, dtype: {cv_image.dtype}"
                )

        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")

    def get_latest_image(self) -> np.ndarray:
        """
        Get the most recently received image.

        Returns:
            numpy.ndarray: The latest image, or None if no image received yet
        """
        return self.latest_image

    def get_image_count(self) -> int:
        """
        Get the total number of images received.

        Returns:
            int: Number of images received
        """
        return self.image_count

    def has_image(self) -> bool:
        """
        Check if at least one image has been received.

        Returns:
            bool: True if an image has been received
        """
        return self.latest_image is not None


class ImageSubscriberNode(Node):
    """
    ROS2 Node for image subscription from Simulink.

    This node creates an ImageSubscriber and runs the main loop
    to process incoming images.
    """

    def __init__(self, topic_name: str = "/simulink/image_raw"):
        """
        Initialize the ImageSubscriberNode.

        Args:
            topic_name: Topic name to subscribe to
        """
        super().__init__('image_subscriber_node')
        self.get_logger().info("ImageSubscriberNode starting...")

        # Create the image subscriber
        self.image_subscriber = ImageSubscriber(self, topic_name)

        # Create a timer for periodic status updates (optional)
        self.status_timer = self.create_timer(5.0, self.status_callback)

        self.get_logger().info("ImageSubscriberNode initialized successfully")

    def status_callback(self):
        """
        Periodic callback to report status.
        """
        count = self.image_subscriber.get_image_count()
        has_img = self.image_subscriber.has_image()
        self.get_logger().info(
            f"Status: received {count} images, "
            f"has_latest_image: {has_img}"
        )

    def get_subscriber(self) -> ImageSubscriber:
        """
        Get the ImageSubscriber instance.

        Returns:
            ImageSubscriber: The image subscriber instance
        """
        return self.image_subscriber


def main(args=None):
    """
    Main entry point for the image subscriber node.
    """
    rclpy.init(args=args)

    # Create the node
    node = ImageSubscriberNode()

    try:
        node.get_logger().info("ImageSubscriberNode running, waiting for images...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
