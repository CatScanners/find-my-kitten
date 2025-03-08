#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


def detect_balls(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray,5)

    # TODO: test and tune these parameters
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=2, minDist=100,
                               param1=50, param2=30, minRadius=0, maxRadius=200)

    ret_circles = np.array([[]])
    if circles is not None:
        ret_circles = np.around(circles)

    return ret_circles


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter("input_topic_name", "image_topic")
        self.declare_parameter("output_topic_name", "detected_objects_topic")
        
        self.input_topic_name = self.get_parameter("input_topic_name").value
        self.output_topic_name = self.get_parameter("output_topic_name").value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            self.input_topic_name,
            self.image_callback,
            10
        )

        # Publisher for detected objects
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            self.output_topic_name,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            circles = detect_balls(cv_image)

            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for (x, y, r) in circles[0, :]:

                detection_msg = Detection2D()
                detection_msg.bbox.center.position.x = x
                detection_msg.bbox.center.position.y = y
                detection_msg.bbox.size_x = 2 * r
                detection_msg.bbox.size_y = 2 * r

                cls = "ball"
                # TODO: give some more realistic confidence estimate
                conf = 1.0

                # Add object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(cls)
                hypothesis.hypothesis.score = float(conf)
                detection_msg.results.append(hypothesis)

                detection_array.detections.append(detection_msg)
                self.get_logger().info(f"Detected: {self.model.names[int(cls)]} {conf:.2f} at ({x}, {y}), with radius {r})")

            # Publish detection results
            self.detection_pub.publish(detection_array)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
