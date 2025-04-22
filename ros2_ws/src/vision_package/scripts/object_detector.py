#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import warnings
warnings.filterwarnings("ignore")


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        self.declare_parameter("input_topic_name", "image_topic")
        self.declare_parameter("output_topic_name", "detected_objects_topic")
        
        self.input_topic_name = self.get_parameter("input_topic_name").value
        self.output_topic_name = self.get_parameter("output_topic_name").value


        # Load the PyTorch model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5s.pt')  # Replace with your .pt file path
        self.model.conf = 0.5  # Confidence threshold

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
            results = self.model(cv_image)

            # Parse detection results
            detections = results.xyxy[0].cpu().numpy()  # Get detections as numpy array

            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for detection in detections:
                x1, y1, x2, y2, conf, cls = detection
                detection_msg = Detection2D()
                detection_msg.bbox.center.position.x = (x1 + x2) / 2.0
                detection_msg.bbox.center.position.y = (y1 + y2) / 2.0
                detection_msg.bbox.size_x = float(x2 - x1)
                detection_msg.bbox.size_y = float(y2 - y1)
                
                # Add object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(cls)
                hypothesis.hypothesis.score = float(conf)
                detection_msg.results.append(hypothesis)
                
                detection_array.detections.append(detection_msg)
                self.get_logger().info(f"Detected: {self.model.names[int(cls)]} {conf:.2f} at ({x1}, {y1}, {x2}, {y2})")

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
