#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from pathlib import Path
from yolov5.models.common import DetectMultiBackend  # Required for local model loading

class YoloObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')

        # Path to the local YOLO model (.pt file)
        model_path = Path("/home/jalowpeura/workspaces/find-my-kitten/ros2_ws/src/vision_package/vision_package/yolov5s.pt")  # Update this path!

        # Select device (CPU or CUDA)
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Load YOLO model without ultralytics dependency
        self.model = DetectMultiBackend(model_path, device=device)  # ✅ Fixed
        self.model.eval()

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10  # QoS
        )

        cv2.namedWindow("Detected Image", cv2.WINDOW_NORMAL)

        self.get_logger().info(f"Loaded local YOLO model from {model_path} on {device}")

    def image_callback(self, msg):
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert image to tensor (YOLO expects torch tensor input)
            img_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).float().unsqueeze(0) / 255.0
            img_tensor = img_tensor.to(self.model.device)  # Move tensor to the correct device

            pred = self.model(img_tensor)  # Run model inference correctly
            detections = pred[0].cpu().numpy()  # Convert to NumPy (first batch)


            for det in detections:
                if len(det) < 6:  # Ensure detection has expected format
                    continue
                x_min, y_min, x_max, y_max, confidence, class_id = det[:6]

                label = f"{self.model.names[int(class_id)]} {confidence:.2f}"

                # Draw bounding box and label
                cv2.rectangle(cv_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (int(x_min), int(y_min) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                self.get_logger().info(f"Detected: {label} at ({x_min}, {y_min}) - ({x_max}, {y_max})")

            # Display the processed image
            cv2.imshow("Detected Image", cv_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

