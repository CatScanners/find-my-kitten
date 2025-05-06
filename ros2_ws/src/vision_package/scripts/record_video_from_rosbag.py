import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.declare_parameter('image_topic', '/image_topic')
        self.declare_parameter('output_file', 'output_video.avi')
        self.declare_parameter('fps', 30.0)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.video_writer = None
        self.get_logger().info(f"Recording from {self.image_topic} to {self.output_file}")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Initialize VideoWriter when first frame arrives
        if self.video_writer is None:
            height, width, _ = cv_image.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(self.output_file, fourcc, self.fps, (width, height))
            self.get_logger().info(f"VideoWriter started: {width}x{height} @ {self.fps} FPS")

        # Write frame to video
        self.video_writer.write(cv_image)

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info("Video file saved and closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, stopping video recorder.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
