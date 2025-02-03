#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Create a publisher for the Image message
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)

        # Set up a timer to publish images periodically
        self.timer = self.create_timer(1, self.timer_callback)

        # URL of the image to download
        self.image_url =  'https://images6.fanpop.com/image/photos/39000000/Cat-cats-39082303-1680-1050.jpg' # Replace with your image URL

        # Download the image from the URL
        self.image = self.download_image(self.image_url)
        if self.image is None:
            self.get_logger().error("Failed to download image from URL.")
            raise ValueError("Failed to download image from URL.")

        # Initialize CV Bridge
        self.bridge = CvBridge()

    def download_image(self, url):
        """
        Download an image from a URL and convert it to an OpenCV image.
        """
        try:
            # Send a GET request to the URL
            response = requests.get(url)
            response.raise_for_status()  # Raise an exception for HTTP errors

            # Convert the response content to a NumPy array
            image_data = np.frombuffer(response.content, dtype=np.uint8)

            # Decode the image using OpenCV
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return image
        except Exception as e:
            self.get_logger().error(f"Error downloading image: {e}")
            return None

    def timer_callback(self):
        if self.image is not None:
            # Convert the OpenCV image to a ROS Image message
            try:
                image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = "camera_frame"  # Set a frame ID (optional)

                # Publish the Image message
                self.publisher_.publish(image_msg)

                self.get_logger().info("Publishing image")
            except Exception as e:
                self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        image_publisher = ImagePublisher()
        rclpy.spin(image_publisher)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
