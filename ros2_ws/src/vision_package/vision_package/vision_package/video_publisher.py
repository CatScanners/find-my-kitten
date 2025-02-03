#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Create a publisher for the CompressedImage message
        self.publisher_ = self.create_publisher(CompressedImage, 'image_topic', 10)

        self.timer = self.create_timer(1, self.timer_callback)

        # Load an example image (replace with your actual image source)
        self.image = cv2.imread('/home/jalowpeura/cat.jpg')
        if self.image is None:
            self.get_logger().warn("Could not load image. Please check the path.")

    def timer_callback(self):
        if self.image is not None:
            # Encode the image as a JPEG compressed image
            ret, encoded_image = cv2.imencode('.jpg', self.image)
            if not ret:
                self.get_logger().error("Failed to encode image")
                return

            # Convert the compressed image into a ROS CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.format = 'jpeg'  # Specify the format as JPEG
            compressed_image_msg.data = encoded_image.tobytes()  # Store the compressed image data

            # Publish the CompressedImage message
            self.publisher_.publish(compressed_image_msg)

            self.get_logger().info("Publishing compressed image")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    # Spin the node to keep the publisher running
    rclpy.spin(image_publisher)

    # Shutdown ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()

