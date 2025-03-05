#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import supervision as sv
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from supervision.detection.core import Detections

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')

        # Initialize the tracker
        self.tracker = sv.ByteTrack()

        # Subscriber to the detected objects topic
        self.subscription = self.create_subscription(
            Detection2DArray,  # Use Detection2DArray
            'detected_objects_topic',
            self.detection_callback,
            10
        )

        # Publisher to the tracked objects topic
        self.publisher = self.create_publisher(
            Detection2DArray,  # Publish tracked objects as Detection2DArray
            'tracked_objects_topic',
            10
        )

    def detections_to_supervision_format(self, detected_objects):
        """
        Convert Detection2DArray messages to a supervision.Detections object.
        """
        xyxy = []
        confidences = []
        class_ids = []
        
        for detected_object in detected_objects.detections:
            if not detected_object.results:
                continue  # Skip detections with no results
            
            bbox = detected_object.bbox
            class_id = detected_object.results[0].hypothesis.class_id
            confidence = detected_object.results[0].hypothesis.score
            
            x1 = bbox.center.position.x - bbox.size_x / 2
            y1 = bbox.center.position.y - bbox.size_y / 2
            x2 = bbox.center.position.x + bbox.size_x / 2
            y2 = bbox.center.position.y + bbox.size_y / 2
            
            xyxy.append([x1, y1, x2, y2])
            confidences.append(confidence)
            class_ids.append(class_id)
        
        if not xyxy:
            return None  # Return None if no valid detections are present

        return Detections(xyxy=np.array(xyxy), confidence=np.array(confidences), class_id=np.array(class_ids))

    def tracked_detections_to_msg(self, tracked_detections, header):
        """
        Convert tracked detections back to Detection2DArray format.
        """
        tracked_objects_msg = Detection2DArray()
        tracked_objects_msg.header = header

        for i in range(len(tracked_detections.xyxy)):
            x1, y1, x2, y2 = tracked_detections.xyxy[i]
            class_id = tracked_detections.class_id[i]
            confidence = tracked_detections.confidence[i]
            tracker_id = tracked_detections.tracker_id[i]
            
            tracked_object = Detection2D()
            tracked_object.bbox.center.position.x = float((x1 + x2) / 2.0)
            tracked_object.bbox.center.position.y = float((y1 + y2) / 2.0)
            tracked_object.bbox.size_x = float(x2 - x1)
            tracked_object.bbox.size_y = float(y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = float(confidence)
            tracked_object.results.append(hypothesis)
            
            self.get_logger().info(f"Detected: {class_id}")
            tracked_objects_msg.detections.append(tracked_object)

        return tracked_objects_msg

    def detection_callback(self, msg):
        """
        Callback function for processing detected objects.
        """
        # Convert incoming message to a supervision.Detections object
        detections = self.detections_to_supervision_format(msg)
        if detections is None:
            return  # Skip processing if no valid detections

        # Update tracker with new detections
        tracked_detections = self.tracker.update_with_detections(detections)

        # Convert tracked detections to Detection2DArray message
        tracked_objects_msg = self.tracked_detections_to_msg(tracked_detections, msg.header)

        # Publish the tracked objects
        self.publisher.publish(tracked_objects_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

