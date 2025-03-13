#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray  # Assuming deviation is published as Int32MultiArray

class TargetDeviationVisualizer(Node):
    def __init__(self):
        super().__init__('target_deviation_visualizer')

        # Declare parameters
        self.declare_parameter("input_images", "image_topic")
        self.declare_parameter("input_deviation", "target_deviation_topic")
        self.declare_parameter("output_topic_name", "visualized_image_topic")

        # Get parameters
        self.input_images = self.get_parameter("input_images").value
        self.input_deviation = self.get_parameter("input_deviation").value
        self.output_topic_name = self.get_parameter("output_topic_name").value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the image and deviation topics
        self.image_sub = self.create_subscription(
            Image,
            self.input_images,
            self.image_callback,
            10
        )

        self.deviation_sub = self.create_subscription(
            Int32MultiArray,  # Assuming deviation is published as Int32MultiArray
            self.input_deviation,
            self.deviation_callback,
            10
        )

        # Publisher for the visualized image
        self.image_pub = self.create_publisher(
            Image,
            self.output_topic_name,
            10
        )

        self.latest_image = None
        self.latest_deviation = None  # Store the latest deviation data

    def image_callback(self, msg):
        """Store the latest received image."""
        self.latest_image = msg
        self.process_and_publish()

    def deviation_callback(self, msg):
        """Store the latest deviation data."""
        self.latest_deviation = msg
        self.process_and_publish()

    def process_and_publish(self):
        """Process the latest image and draw a line based on deviation data."""
        if self.latest_image is None or self.latest_deviation is None:
            return

        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        # Get image dimensions
        height, width, _ = cv_image.shape
        screen_center_x = width // 2
        screen_center_y = height // 2

        # Extract deviation data (assuming it contains [x, y] coordinates)
        if len(self.latest_deviation.data) >= 2:
            target_x = self.latest_deviation.data[0]
            target_y = self.latest_deviation.data[1]

            # Draw a line from the screen center to the target position
            cv2.arrowedLine(
                cv_image,
                (screen_center_x, screen_center_y),  # Start from screen center
                (target_x, target_y),  # End at target position
                (0, 0, 255),  # Red color
                2,  # Thickness
                tipLength=0.1
            )

            # Optionally, display the deviation values
            deviation_x = target_x - screen_center_x
            deviation_y = target_y - screen_center_y
            cv2.putText(
                cv_image,
                f'Deviation: X={deviation_x}, Y={deviation_y}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),  # White color
                2
            )

        # Convert back to ROS Image message and publish
        visualized_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(visualized_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDeviationVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
