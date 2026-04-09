#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("point_odometry")
    {
        // callback lamba function
        // tämän sisään tulee Lassin koodi, msg tyypit täytyy vielä muuttaa oikeiksi

        auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void
        {
            
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            auto message = std_msgs::msg::String();

            message.data = std::to_string(std::stoi(msg->data) * 2);
            this->publisher_->publish(message);
        };

        // TODO: päivitä oikeat topic nimet
        subscription_ = this->create_subscription<std_msgs::msg::String>("detections", 10, topic_callback);
        // publisher that publishes point odometry messages to point_odometry topic 
        publisher_ = this->create_publisher<std_msgs::msg::String>("point_odometry", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}