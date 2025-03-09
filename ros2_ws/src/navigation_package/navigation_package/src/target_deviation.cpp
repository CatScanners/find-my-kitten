#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

class TargetDeviation : public rclcpp::Node
{
public:
    std::string input_track;
    int res_width;
    int res_height;
    std::string output_topic_name;

    // Constructor
    TargetDeviation() : Node("target_deviation")
    {
        // Declare parameters with defaults
        this->declare_parameter("input_track", "tracked_objects_topic");
        this->declare_parameter("res_width", 1920);
        this->declare_parameter("res_height", 1080);
        this->declare_parameter("output_topic_name", "target_deviation_topic");

        // Get parameters
        this->get_parameter("input_track", input_track);
        this->get_parameter("res_width", res_width);
        this->get_parameter("res_height", res_height);
        this->get_parameter("output_topic_name", output_topic_name);

        // Create subscriber
        subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            input_track,
            10,
            std::bind(&TargetDeviation::calculateDeviation, this, std::placeholders::_1));

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_name, 10);
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    void calculateDeviation(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        if (msg->detections.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No tracked objects received.");
            return;
        }
        // Take the first detected object
        auto target = msg->detections[0];
        float object_x = target.bbox.center.position.x;
        float object_y = target.bbox.center.position.y;

        // Calculate deviation from screen center
        float center_x = res_width / 2.0;
        float center_y = res_height / 2.0;
        float deviation_x = (object_x - center_x );
        float deviation_y = (object_y - center_y );

        // Publish deviation as FLoat32MultiArray
        auto deviation_msg = std_msgs::msg::Float32MultiArray();
        deviation_msg.data = {deviation_x, deviation_y};
        publisher_->publish(deviation_msg);

        RCLCPP_INFO(this->get_logger(), "Normalized Coordinates -> X: %.2f, Y: %.2f", deviation_x, deviation_y);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDeviation>());
    rclcpp::shutdown();
    return 0;
}
