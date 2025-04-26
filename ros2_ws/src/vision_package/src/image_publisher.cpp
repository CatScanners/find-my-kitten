#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <string>

class ImagePublisher: public rclcpp::Node
{
    public:
	std::string topic_name;
        int cam_id;
        int res_width;
        int res_height;
        double pub_time;
        cv::VideoCapture cap;

        // Constructor
        ImagePublisher(): Node("image_publisher")
        {
            // Declare parameters with defaults
	    this->declare_parameter("topic_name", "image_topic");
            this->declare_parameter("camera_id", 0);
            this->declare_parameter("res_width", 1920);
            this->declare_parameter("res_height", 1080);
            this->declare_parameter("pub_time", 0.03);

            // Get parameters
	    this->get_parameter("topic_name", topic_name);
            this->get_parameter("camera_id", cam_id);
            this->get_parameter("res_width", res_width);
            this->get_parameter("res_height", res_height);
            this->get_parameter("pub_time", pub_time);

            // Open video stream
            cap = open_stream(cam_id);

            // Create the image publisher and timer
            _image_publisher_ = image_transport::create_publisher(this, topic_name);
            _image_timer_ = this->create_wall_timer(std::chrono::duration<double>(pub_time), std::bind(&ImagePublisher::publish_frame, this));
        }

    private:
        image_transport::Publisher _image_publisher_;
        rclcpp::TimerBase::SharedPtr _image_timer_;
        
        // Opens the camera stream and sets to high resolution.
        cv::VideoCapture open_stream(int cam_id)
        {
            cap.open(cam_id, cv::CAP_V4L2); // CAP_V4L2 is Linux only.
            if (!cap.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Camera could not be opened on device id: '%i'", cam_id);
                exit(0);
            }

            RCLCPP_INFO(this->get_logger(), "Camera opened on device id: '%i'", cam_id);

            // Set video properties
            cap.set(3, res_width);
            cap.set(4, res_height);

            return cap;
        }

        void publish_frame()
        {
            // Read camera frame
            cv::Mat frame;
            cap.read(frame);
            if (frame.empty()){
                RCLCPP_WARN(this->get_logger(), "Frame data is empty");
                return;
            }

            // Create ROS2 message using cv_bridge
            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            cv_bridge::CvImage cv_bridge_image(header, sensor_msgs::image_encodings::BGR8, frame);

            // Publish the image
            _image_publisher_.publish(cv_bridge_image.toImageMsg());
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
