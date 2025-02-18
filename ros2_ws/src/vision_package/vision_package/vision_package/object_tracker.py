#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import supervision as sv
from your_custom_msgs.msg import DetectedObjects, TrackedObjects  # Replace with your custom message types

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')
        
        # Initialize the tracker
        self.tracker = sv.ByteTrack()

        # Subscriber to the detected objects topic
        self.subscription = self.create_subscription(
            DetectedObjects,  # Replace with your custom message type
            'detected_objects_topic',
            self.detection_callback,
            10
        )

        # Publisher to the tracked objects topic
        self.publisher = self.create_publisher(
            TrackedObjects,  # Replace with your custom message type
            'tracked_objects_topic',
            10
        )

    def detections_to_detected_objects(self, detections):
        """
        Convert supervision Detections object to a list of DetectedObjects messages.
        """
        detected_objects = []
        for detection in detections:
            detected_object = DetectedObjects()  # Replace with your custom message type
            detected_object.bbox = [detection[0], detection[1], detection[2], detection[3]]  # x1, y1, x2, y2
            detected_object.class_id = detection[4]  # Class ID
            detected_object.confidence = detection[5]  # Confidence score
            detected_objects.append(detected_object)
        return detected_objects

    def detections_to_tracked_objects(self, detections):
        """
        Convert supervision Detections object to a list of TrackedObjects messages.
        """
        tracked_objects = []
        for detection in detections:
            tracked_object = TrackedObjects()  # Replace with your custom message type
            tracked_object.bbox = [detection[0], detection[1], detection[2], detection[3]]  # x1, y1, x2, y2
            tracked_object.class_id = detection[4]  # Class ID
            tracked_object.tracker_id = detection[6]  # Tracker ID
            tracked_objects.append(tracked_object)
        return tracked_objects

    def detections_to_supervision_format(self, detected_objects):
        """
        Convert DetectedObjects messages to a format compatible with supervision.
        """
        detections = []
        for detected_object in detected_objects:
            detections.append([
                detected_object.bbox[0],  # x1
                detected_object.bbox[1],  # y1
                detected_object.bbox[2],  # x2
                detected_object.bbox[3],  # y2
                detected_object.class_id,  # Class ID
                detected_object.confidence  # Confidence score
            ])
        return np.array(detections)

    def detection_callback(self, msg):
        """
        Callback function for processing detected objects.
        """
        # Convert incoming message to a format compatible with supervision
        detections = self.detections_to_supervision_format(msg.detected_objects)

        # Update tracker with new detections
        tracked_detections = self.tracker.update_with_detections(detections)

        # Convert tracked detections to TrackedObjects messages
        tracked_objects = self.detections_to_tracked_objects(tracked_detections)

        # Publish the tracked objects
        tracked_objects_msg = TrackedObjects()
        tracked_objects_msg.objects = tracked_objects
        self.publisher.publish(tracked_objects_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
