#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class TrackedImagePublisher(Node):
    def __init__(self):
        super().__init__('tracked_image_publisher')
       

        self.declare_parameter("input_images", "image_topic")
        self.declare_parameter("input_tracks", "tracked_objects_topic")
        self.declare_parameter("output_topic_name", "tracked_image_topic")
        
        self.input_images = self.get_parameter("input_images").value
        self.input_tracks = self.get_parameter("input_tracks").value
        self.output_topic_name = self.get_parameter("output_topic_name").value

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to the image and tracked objects topics
        self.image_sub = self.create_subscription(
            Image,
            self.input_images,
            self.image_callback,
            10
        )
        
        self.tracked_objects_sub = self.create_subscription(
            Detection2DArray,
            self.input_tracks,
            self.tracked_objects_callback,
            10
        )
        
        # Publisher for the annotated image
        self.image_pub = self.create_publisher(
            Image,
            self.output_topic_name,
            10
        )
        
        self.latest_image = None
        self.latest_tracked_objects = None
    
    def image_callback(self, msg):
        """ Store the latest received image """
        self.latest_image = msg
        self.process_and_publish()
    
    def tracked_objects_callback(self, msg):
        """ Store the latest tracked objects """
        self.latest_tracked_objects = msg
        self.process_and_publish()
    
    def process_and_publish(self):
        """ Process the latest image and overlay tracked objects before publishing. """
        if self.latest_image is None or self.latest_tracked_objects is None:
            return
        
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        
        # Draw bounding boxes and object IDs
        for detection in self.latest_tracked_objects.detections:
            x = int(detection.bbox.center.position.x - detection.bbox.size_x / 2)
            y = int(detection.bbox.center.position.y - detection.bbox.size_y / 2)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)
            
            tracker_id = "Unknown"
            if detection.results:
                tracker_id = str(detection.results[0].hypothesis.class_id)
            
            # Draw rectangle
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Put text (Object ID)
            cv2.putText(cv_image, f'ID: {tracker_id}', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Convert back to ROS Image message and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackedImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
