#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class DeviationVisualizer(Node):
    def __init__(self):
        super().__init__('deviation_visualizer')

        # Declare parameters
        self.declare_parameter("input_image_topic", "tracked_image_topic")
        self.declare_parameter("deviation_topic", "target_deviation_topic")
        self.declare_parameter("output_image_topic", "deviation_visualized_topic")

        # Get parameter values
        self.input_image_topic = self.get_parameter("input_image_topic").value
        self.deviation_topic = self.get_parameter("deviation_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to tracked images and deviation data
        self.image_sub = self.create_subscription(
            Image, self.input_image_topic, self.image_callback, 10)

        self.deviation_sub = self.create_subscription(
            Float32MultiArray, self.deviation_topic, self.deviation_callback, 10)

        # Publisher for the visualized deviation image
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        # Store latest received deviation
        self.latest_deviation = None

    def deviation_callback(self, msg):
        """Store latest deviation values."""
        if len(msg.data) >= 2:
            self.latest_deviation = (int(msg.data[0]), int(msg.data[1]))

    def image_callback(self, msg):
        """Draw deviation vector on image and publish."""
        if self.latest_deviation is None:
            return

        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get image dimensions
        height, width, _ = cv_image.shape

        # Define center of the image
        center_x, center_y = width // 2, height // 2

        # Extract deviation values
        deviation_x, deviation_y = self.latest_deviation

        # Define endpoint of the deviation vector
        vector_end_x = center_x + deviation_x
        vector_end_y = center_y + deviation_y

        # Draw X-Y axis (coordinate plane)
        cv2.line(cv_image, (center_x, 0), (center_x, height), (255, 0, 0), 2)  # Vertical axis (Y)
        cv2.line(cv_image, (0, center_y), (width, center_y), (255, 0, 0), 2)  # Horizontal axis (X)

        # Draw deviation vector
        cv2.arrowedLine(cv_image, (center_x, center_y), (vector_end_x, vector_end_y),
                        (0, 0, 255), 3, tipLength=0.2)

        # Convert back to ROS Image message and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeviationVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
