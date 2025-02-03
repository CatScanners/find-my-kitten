#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <vision_package/variables.hpp>

class ImagePublisher: public rclcpp::Node
{

    public:
        int cam_id;
        cv::VideoCapture cap;

        //constructor
        ImagePublisher():Node("image_publisher")
        {

            // declare parameters & open video stream
            this -> get_parameter(vision_package::RGB_CAMERA_ID, cam_id);
            cap = open_stream(0);

            //create the image publisher and timer
            _image_publisher_ = image_transport::create_publisher(this, vision_package::RGB_IMAGE_TOPIC);
            _image_timer_ = this -> create_wall_timer(vision_package::RGB_PUB_TIME, std::bind(&ImagePublisher::publish_frame, this));

        }


        private:
        image_transport::Publisher _image_publisher_;
        rclcpp::TimerBase::SharedPtr _image_timer_;
        
        //Opens the camera stream and sets to high resolution.
        cv::VideoCapture open_stream(int cam_id)
        {
            cap.open(cam_id, cv::CAP_V4L2); //CAP_V4L2 is linux only.
            if (!cap.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Camera could not be opened on device id: '%i'", cam_id);
                exit(0);
            }

            RCLCPP_INFO(this->get_logger(), "Camera opened on device id: '%i'", cam_id);

            //set video properties. 3 width and 4 is heigth
            cap.set(3, vision_package::RGB_RES_WIDTH);
            cap.set(4, vision_package::RGB_RES_HEIGHT);

            return cap;
        }

        void publish_frame()
        {
            //read camera frame
            cv::Mat frame;
            cap.read(frame);
            if (frame.empty()){
                RCLCPP_WARN(this->get_logger(), "Frame data is emtpy");
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
