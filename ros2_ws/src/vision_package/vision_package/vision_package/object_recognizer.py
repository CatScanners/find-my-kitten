#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Load the PyTorch model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5s.pt')  # Replace with your .pt file path
        self.model.conf = 0.5  # Confidence threshold

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            'image_topic',  # Replace with your image topic name
            self.image_callback,
            10
        )

        # Publisher for detected objects
        self.detection_pub = self.create_publisher(
            Image,  # Replace with a custom message type if needed
            'detected_objects_topic',  # Replace with your output topic name
            10
        )

        # OpenCV window for displaying images
        cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            results = self.model(cv_image)

            # Parse detection results
            detections = results.xyxy[0].cpu().numpy()  # Get detections as numpy array

            # Draw bounding boxes and labels on the image
            for detection in detections:
                x1, y1, x2, y2, conf, cls = detection
                label = f"{self.model.names[int(cls)]} {conf:.2f}"
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                # Publish detected objects (customize this part as needed)
                self.get_logger().info(f"Detected: {label} at ({x1}, {y1}, {x2}, {y2})")

            # Display the image with bounding boxes
            cv2.imshow('Object Detection', cv_image)
            cv2.waitKey(1)

            # Publish the annotated image (optional)
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.detection_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
