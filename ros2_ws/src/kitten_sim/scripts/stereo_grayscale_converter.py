#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class StereoGrayscaleConverter(Node):
    def __init__(self):
        super().__init__('stereo_grayscale_converter')
        self.bridge = CvBridge()
        
        # Use sensor-compatible QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub_left = self.create_subscription(
            Image, '/drone1/stereo_left_camera/color/image_raw', 
            self.left_cb, sensor_qos)
        self.sub_right = self.create_subscription(
            Image, '/drone1/stereo_right_camera/color/image_raw', 
            self.right_cb, sensor_qos)
        
        self.pub_left = self.create_publisher(Image, '/drone1/stereo_left_camera/grayscale/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/drone1/stereo_right_camera/grayscale/image_raw', 10)
        
        self.get_logger().info('Stereo RGB→Grayscale node started')

    def left_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Handle both color and mono images
            if len(cv_img.shape) == 3:
                gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv_img
                
            gray_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
            gray_msg.header = msg.header
            self.pub_left.publish(gray_msg)
            
        except Exception as e:
            self.get_logger().error(f'Left callback error: {e}')

    def right_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if len(cv_img.shape) == 3:
                gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv_img
                
            gray_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
            gray_msg.header = msg.header
            self.pub_right.publish(gray_msg)
            
        except Exception as e:
            self.get_logger().error(f'Right callback error: {e}')


def main():
    rclpy.init()
    node = StereoGrayscaleConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
