#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import CompressedImage
import sys

# Why python - OpenCV C++ does not support Pytorch (.pt) files. Alternative formats like ONNX are suported by openCV but all pretrained models are offered in .pt by default. 

class YoloObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')
        
        # Load the YOLO model (assuming it's a pytorch model stored as a .pt file)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.model.eval()

        # Initialize the CV bridge for ROS to OpenCV conversions
        self.bridge = CvBridge()

        # Create a subscriber to the 'image_topic' (you can adjust topic as needed)
        self.image_subscriber = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10  # QoS value (you can adjust it)
        )

        self.get_logger().info("YOLO Object Detection Node has been started.")

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLOv5 model inference
            results = self.model(cv_image)  # Run inference
            
            # Process the results
            detected_objects = results.pandas().xywh[0].to_dict(orient="records")  # Get detection results
            
            # Display results on the image
            for obj in detected_objects:
                # Extract object data (class, confidence, bbox)
                label = obj['name']
                confidence = obj['confidence']
                x_min, y_min, x_max, y_max = int(obj['xmin']), int(obj['ymin']), int(obj['xmax']), int(obj['ymax'])

                # Draw bounding box and label
                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{label} {confidence:.2f}", (x_min, y_min-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Optionally, you can save or display the image
            # cv2.imwrite('output_image.jpg', cv_image)
            # cv2.imshow("Detected Image", cv_image)
            # cv2.waitKey(1)

            # Log the detected objects
            for obj in detected_objects:
                self.get_logger().info(f"Detected object: {obj['name']} with confidence {obj['confidence']:.2f} "
                                       f"at location: ({obj['xmin']}, {obj['ymin']}) - ({obj['xmax']}, {obj['ymax']})")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
