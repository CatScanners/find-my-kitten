#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


# This node finds balls in an image feed, and gives a list of bounding boxes in pixel coordinates.

def detect_balls(image):# , param1, param2):
    wimg, himg = image.shape[:2]
    avgmetric = 2/(wimg + himg)
    # Size to approx. 500 x 500
    avg_pixel_dim = 500
    h = int(avgmetric * avg_pixel_dim * wimg)
    w = int(avgmetric * avg_pixel_dim * himg)
    inv_resize = wimg / w
    image_resized = cv2.resize(image, (w, h))

    # Convert the image to the HSV color space:
    hsvImage = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

    # Set the HSV values:
    lowRange = np.array([60, 30, 120])
    uppRange = np.array([270, 255, 255])

    # Mask for high-contrast/value objects
    binmask = cv2.inRange(hsvImage, lowRange, uppRange)
    # Apply Dilate + Erode:
    kernel = np.ones((3,3), np.uint8)
    binmask = cv2.morphologyEx(binmask, cv2.MORPH_DILATE, kernel, iterations=1)

    masked_image = cv2.bitwise_and(image_resized, image_resized, mask=binmask)

    binmask = cv2.medianBlur(binmask, 9)
    cv2.imshow("binmask", binmask)
    cv2.imshow("mask", masked_image)

    # Find contours
    contours, _ = cv2.findContours(binmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_circularity = 0.85
    min_area = 0.02 * 0.02 * w*h/2

    circles = [[]]

    for contour in contours:
        # Approximate the contour to a polygon
        area = cv2.contourArea(contour)
        # Filter out what's less than 2% width / height.
        if area < min_area:
            continue
        perimeter = cv2.arcLength(contour, True)

        circularity = (4 * 3.1416 * area)/(perimeter**2)

        # Check if the polygon has fewer than 6 vertices (as circles are smooth)
        if circularity > min_circularity:  # A circle should have many points
            # Get the center and radius of the detected circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            circles[0].append([int(x), int(y), int(radius)])
            cv2.circle(image_resized, (int(x), int(y)), int(radius), (0, 255, 0), 2)

    circles = (np.array(circles) * inv_resize)

    ret_circles = np.around(circles)

    return ret_circles

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter("input_topic_name", "image_topic")
        self.declare_parameter("output_topic_name", "detected_objects_topic")

        # Expose hough parameters
        # self.declare_parameter("hough_param_1", 200)
        # self.declare_parameter("hough_param_2", 80)

        # self.hough_param_1 = self.get_parameter('hough_param_1').value
        # self.hough_param_2 = self.get_parameter('hough_param_2').value

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
            circles = detect_balls(cv_image) # , param1=self.hough_param_1, param2=self.hough_param_2)

            detection_array = Detection2DArray()
            detection_array.header = msg.header

            self.get_logger().info("Before for loop")
            self.get_logger().info(f"Circles: {circles}")
            for (x, y, r) in circles[0, :]:
                self.get_logger().info("In for loop")
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
                self.get_logger().info(f"Detected: {conf:.2f} at ({x}, {y}), with radius {r})")

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
