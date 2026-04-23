#!/usr/bin/env python3

import warnings

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

warnings.filterwarnings("ignore")


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self.declare_parameter("image_topic", "/argus/left/image_raw")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("dimension_topic", "/dimensions")
        self.declare_parameter("yolo_file", None)

        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.dimension_topic = self.get_parameter("dimension_topic").value
        self.yolo_file = self.get_parameter("yolo_file").value

        assert self.yolo_file is not None, (
            "Must give argument for yolo_file to take the yolo model from"
        )
        self.model = YOLO(self.yolo_file, task="detect")

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        # Publisher for detected objects
        self.detection_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10
        )
        dimension_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dimension_pub = self.create_publisher(
            Int32MultiArray, self.dimension_topic, dimension_qos
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = cv_image.shape[:2]
            dimension_msg = Int32MultiArray()
            dimension_msg.data = [int(width), int(height)]
            self.dimension_pub.publish(dimension_msg)

            # Perform object detection
            results = self.model.track(
                cv_image,
                persist=True,
                conf=0.3,
                verbose=False,
            )
            # We get just one image each time, so only one result
            boxes = results[0].cpu().boxes
            if boxes is None or len(boxes) == 0:
                return

            xywh_coords = boxes.xywh
            confidences = boxes.conf
            detected_classes = boxes.cls
            tracking_ids = boxes.id

            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for box_idx in range(len(boxes)):
                center_x, center_y, box_width, box_height = xywh_coords[box_idx]
                tracking_id = (
                    tracking_ids[box_idx] if tracking_ids is not None else None
                )
                confidence = confidences[box_idx]
                detected_class = detected_classes[box_idx]

                detection_msg = Detection2D()
                detection_msg.id = (
                    str(int(tracking_id)) if tracking_id is not None else ""
                )

                detection_msg.bbox.center.position.x = float(center_x)
                detection_msg.bbox.center.position.y = float(center_y)
                detection_msg.bbox.size_x = float(box_width)
                detection_msg.bbox.size_y = float(box_height)

                # Add object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(detected_class))
                hypothesis.hypothesis.score = float(confidence)
                detection_msg.results.append(hypothesis)

                detection_array.detections.append(detection_msg)
                self.get_logger().info(
                    f"Detected: {self.model.names[int(detected_class)]} {confidence:.2f} at ({center_x}, {center_y})"
                )

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


if __name__ == "__main__":
    main()
