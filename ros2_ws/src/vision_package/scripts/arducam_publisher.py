#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Declare parameters
        self.declare_parameter("topic_name", "image_topic")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("res_width", 1920)
        self.declare_parameter("res_height", 1080)
        self.declare_parameter("pub_time", 0.03)

        # Get parameters
        self.topic_name = self.get_parameter("topic_name").value
        self.cam_id = self.get_parameter("camera_id").value
        self.res_width = self.get_parameter("res_width").value
        self.res_height = self.get_parameter("res_height").value
        self.pub_time = self.get_parameter("pub_time").value

        # Set pixel format to BA10 using v4l2-ctl
        self.set_camera_format()

        # Open the camera
        self.cap = cv.VideoCapture(self.cam_id, cv.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.cam_id}")
            exit(1)

        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.res_width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.res_height)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.topic_name, 10)
        self.timer = self.create_timer(self.pub_time, self.publish_frame)

        self.get_logger().info(f"Publishing camera stream on '{self.topic_name}'")

    def set_camera_format(self):
        
        # To make this node work, it is crucial to set V4L2 video format to correct type eg. BA10. 
        # Other wise OpenCV is not able to intreped the video stream with color. 
        cmd = [
            'v4l2-ctl',
            f'-d', f'/dev/video{self.cam_id}',
            '--set-fmt-video',
            f'width={self.res_width},height={self.res_height},pixelformat=BA10'
        ]
        try:
            subprocess.run(cmd, check=True)
            time.sleep(0.5)  # let it apply
            self.get_logger().info("Camera format set to BA10")
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"v4l2-ctl failed: {e}")

    def publish_frame(self):
        ret, bayer_frame = self.cap.read()

        if not ret or bayer_frame is None:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # This is a curiosity of our hardware. Arducam globalshutter camera color format is BA10
        # which in linux is stored in a 16bit buffer with padding. OpenCV on the other hand does not 
        # understand this, especially as color data. 
        # This is why we first nee
        # back to BGR from BayerGB with demosaicing. This is still just a hypothesis and the solution was found by testing 
        # different Bayer formats from which the correct colors were found. 
        
        if bayer_frame.ndim == 3:
            # It got interpreted as BGR, force grayscale
            bayer_frame = cv.cvtColor(bayer_frame, cv.COLOR_BGR2GRAY)

        # Demosaic Bayer to RGB (choose pattern that matches your sensor)
        rgb_frame = cv.demosaicing(bayer_frame, cv.COLOR_BayerGB2BGR)

        msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

