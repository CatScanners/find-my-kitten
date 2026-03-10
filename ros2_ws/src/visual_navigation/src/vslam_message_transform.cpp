#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class VslamMessageTransform : public rclcpp::Node
{
public:
    VslamMessageTransform() : Node("vslam_message_transform")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/visual_slam/tracking/odometry", 10,
            std::bind(&VslamMessageTransform::odometry_callback, this, _1));

        publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto out = px4_msgs::msg::VehicleOdometry();
        std::cout << msg << std::endl;

        publisher_->publish(out);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VslamMessageTransform>());
    rclcpp::shutdown();
    return 0;
}
