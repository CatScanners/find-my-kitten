#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

class TargetDeviation : public rclcpp::Node
{
public:
    std::string input_track;
    int res_width;
    int res_height;
    std::string output_topic_name;
    std::string depth_topic;
    float latest_depth = 1000.0;  // Default depth if none received

    // Constructor
    TargetDeviation() : Node("target_deviation"), res_width(1920), res_height(1080)
    {
        // Declare parameters
        this->declare_parameter("input_track", "tracked_objects_topic");
        this->declare_parameter("output_topic_name", "target_deviation_topic");
        this->declare_parameter("image_dimension_topic", "image_dimension_topic");
        this->declare_parameter("depth_topic", "depth_data_topic");

        // Get parameters
        this->get_parameter("input_track", input_track);
        this->get_parameter("output_topic_name", output_topic_name);
        this->get_parameter("image_dimension_topic", image_dimension_topic);
        this->get_parameter("depth_topic", depth_topic);

        // Subscribe to tracked objects
        subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            input_track, 10,
            std::bind(&TargetDeviation::calculateDeviation, this, std::placeholders::_1));

        // Subscribe to image dimensions
        dimension_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            image_dimension_topic, 10,
            std::bind(&TargetDeviation::updateImageDimensions, this, std::placeholders::_1));

        // Subscribe to depth data
        depth_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            depth_topic, 10,
            std::bind(&TargetDeviation::updateDepth, this, std::placeholders::_1));

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_name, 10);
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr dimension_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    std::string image_dimension_topic;

    void updateImageDimensions(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2)
        {
            res_width = msg->data[0];
            res_height = msg->data[1];
            RCLCPP_INFO(this->get_logger(), "Updated Image Dimensions -> Width: %d, Height: %d", res_width, res_height);
        }
    }

    void updateDepth(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_depth = msg->data;
        RCLCPP_INFO(this->get_logger(), "Updated Depth Data -> Depth: %.2f", latest_depth);
    }

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

        // Image center (camera origin)
        float center_x = res_width / 2.0;
        float center_y = res_height / 2.0;
        float center_z = 1000.0;  // Camera at 1000 units depth

        // Convert pixel deviation to world coordinates
        float deviation_x = (object_x - center_x) * (latest_depth / center_z);
        float deviation_y = (object_y - center_y) * (latest_depth / center_z);
        float deviation_z = latest_depth - center_z;

        // Publish deviation as Float32MultiArray
        auto deviation_msg = std_msgs::msg::Float32MultiArray();
        deviation_msg.data = {deviation_x, deviation_y, deviation_z};
        publisher_->publish(deviation_msg);

        RCLCPP_INFO(this->get_logger(), "3D Deviation -> X: %.2f, Y: %.2f, Z: %.2f", deviation_x, deviation_y, deviation_z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDeviation>());
    rclcpp::shutdown();
    return 0;
}
