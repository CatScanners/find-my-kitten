#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import os
import re
import subprocess

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

    
        self.declare_parameter('input_source', 'https://www.youtube.com/watch?v=dQw4w9WgXcQ')
        self.input_source = self.get_parameter('input_source').value

        # Create a publisher for the Image message
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)

        # Set up a timer to publish images/frames periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer for video frame rate

        # Input can be a URL (image or YouTube video) or a local file path
        self.is_video = False
        self.video_capture = None
        self.image = None

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Process the input source
        self.process_input(self.input_source)

    def process_input(self, input_source):
        """
        Determine if the input is an image, a video file, or a YouTube video.
        """
        if os.path.isfile(input_source):  # Local file
            if input_source.endswith(('.mp4', '.avi', '.mkv')):  # Video file
                self.is_video = True
                self.video_capture = cv2.VideoCapture(input_source)
                if not self.video_capture.isOpened():
                    self.get_logger().error("Failed to open video file.")
                    raise ValueError("Failed to open video file.")
            else:  # Assume it's an image file
                self.image = cv2.imread(input_source)
                if self.image is None:
                    self.get_logger().error("Failed to load image file.")
                    raise ValueError("Failed to load image file.")
        else:  # URL
            if 'youtube.com' in input_source or 'youtu.be' in input_source:  # YouTube video
                self.is_video = True
                self.video_capture = self.download_youtube_video(input_source)
            else:  # Image URL
                self.image = self.download_image(input_source)
                if self.image is None:
                    self.get_logger().error("Failed to download image from URL.")
                    raise ValueError("Failed to download image from URL.")

    def download_image(self, url):
        """
        Download an image from a URL and convert it to an OpenCV image.
        """
        try:
            response = requests.get(url)
            response.raise_for_status()
            image_data = np.frombuffer(response.content, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return image
        except Exception as e:
            self.get_logger().error(f"Error downloading image: {e}")
            return None

    def download_youtube_video(self, url):
        """
        Download a YouTube video using youtube-dl and open it with OpenCV.
        """
        try:
            # Use youtube-dl to get the video URL
            ytdl_command = f"yt-dlp -g -f mp4 {url}"
            video_url = subprocess.check_output(ytdl_command, shell=True).decode().strip()

            # Open the video with OpenCV
            video_capture = cv2.VideoCapture(video_url)
            if not video_capture.isOpened():
                self.get_logger().error("Failed to open YouTube video.")
                raise ValueError("Failed to open YouTube video.")
            return video_capture
        except Exception as e:
            self.get_logger().error(f"Error downloading YouTube video: {e}")
            return None

    def timer_callback(self):
        if self.is_video:
            # Read the next frame from the video
            ret, frame = self.video_capture.read()
            if not ret:
                self.get_logger().info("End of video.")
                self.timer.cancel()  # Stop the timer if the video ends
                return
            self.image = frame

        if self.image is not None:
            # Convert the OpenCV image to a ROS Image message
            try:
                image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = "camera_frame"  # Set a frame ID (optional)

                # Publish the Image message
                self.publisher_.publish(image_msg)

                self.get_logger().info("Publishing image/frame")
            except Exception as e:
                self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        image_publisher = ImagePublisher()
        rclpy.spin(image_publisher)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
