#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("image_subscriber")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("topic_name", "image_topic");
        std::string image_topic = this->get_parameter("topic_name").as_string();

       // Subscribe to the image topic using image_transport
        image_sub_ = image_transport::create_subscription(
            this,
            image_topic,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1),
            "raw", // Use "raw" transport (default)
            rmw_qos_profile_sensor_data
        );

        // Display the window
        cv::namedWindow("RGB Camera Feed", cv::WINDOW_NORMAL);
    }

private:
    // Subscriber for image messages
    image_transport::Subscriber image_sub_;

    // Callback function for receiving and displaying images
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            // Convert the ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Display the image in a window
            cv::imshow("RGB Camera Feed", cv_ptr->image);
            cv::waitKey(1);  // Display image and wait for key press (to refresh the window)
            std::cout << "Received" << std::endl; // Printing received message for debug purposes. 
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}

