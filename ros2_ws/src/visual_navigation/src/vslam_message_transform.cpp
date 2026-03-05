#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <math.h>

#define LOG(severity, fmt, ...) \
    RCLCPP_##severity(this->get_logger(), fmt, ##__VA_ARGS__)

using std::placeholders::_1;

/*
* The purpose of this node is to bridge the odometry output from IsaacROS vslam
* to PX4's EKF2. This means transforming the values and publishing it as another
* message type.
* From type: https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
* To type: https://docs.px4.io/main/en/msg_docs/VehicleOdometry
*/
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
        auto pos = msg->pose.pose.position;
        auto quat = msg->pose.pose.orientation;
        tf2::Vector3 position(pos.x, pos.y, pos.z);
        tf2::Quaternion attitude(quat.x, quat.y, quat.z, quat.w);

        auto linear = msg->twist.twist.linear;
        auto angular = msg->twist.twist.angular;
        tf2::Vector3 velocity(linear.x, linear.y, linear.z);
        tf2::Vector3 angular_velocity(angular.x, angular.y, angular.z);

        // The indices are the diagonals of a 6x6 matrix
        // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
        // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html
        std::array<double, 36> pose_covar = msg->pose.covariance;
        std::array<double, 36> twist_covar = msg->twist.covariance;
        tf2::Vector3 position_variance(pose_covar[0], pose_covar[7], pose_covar[14]);
        tf2::Vector3 orientation_variance(pose_covar[21], pose_covar[28], pose_covar[35]);
        tf2::Vector3 velocity_variance(twist_covar[0], twist_covar[7], twist_covar[14]);

        // the vslam uses the axes x=forward, y=left, z=up.
        // PX4 wants x=forward, y=right, z=down
        // So we rotate everything 180 degrees around the X axis
        tf2::Quaternion rotation;
        rotation.setRPY(M_PI, 0, 0);
        position = tf2::quatRotate(rotation, position);
        attitude = rotation * attitude * rotation.inverse();

        velocity = tf2::quatRotate(rotation, velocity);
        angular_velocity = tf2::quatRotate(rotation, angular_velocity);

        position_variance = tf2::quatRotate(rotation, position_variance);
        orientation_variance = tf2::quatRotate(rotation, orientation_variance);
        velocity_variance = tf2::quatRotate(rotation, velocity_variance);

        px4_msgs::msg::VehicleOdometry out = px4_msgs::msg::VehicleOdometry();
        out.timestamp = now().nanoseconds() / 1'000;
        out.timestamp_sample = 
            static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000
            + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1'000;

        out.pose_frame = out.POSE_FRAME_FRD;
        out.position[0] = position.getX();
        out.position[1] = position.getY();
        out.position[2] = position.getZ();
        out.q[0] = attitude.getW();
        out.q[1] = attitude.getX();
        out.q[2] = attitude.getY();
        out.q[3] = attitude.getZ();

        out.velocity_frame = out.VELOCITY_FRAME_BODY_FRD;
        out.velocity[0] = velocity.getX();
        out.velocity[1] = velocity.getY();
        out.velocity[2] = velocity.getZ();
        out.angular_velocity[0] = angular_velocity.getX();
        out.angular_velocity[1] = angular_velocity.getY();
        out.angular_velocity[2] = angular_velocity.getZ();

        out.position_variance[0] = position_variance.getX();
        out.position_variance[1] = position_variance.getY();
        out.position_variance[2] = position_variance.getZ();
        out.orientation_variance[0] = orientation_variance.getX();
        out.orientation_variance[1] = orientation_variance.getY();
        out.orientation_variance[2] = orientation_variance.getZ();
        out.velocity_variance[0] = velocity_variance.getX();
        out.velocity_variance[1] = velocity_variance.getY();
        out.velocity_variance[2] = velocity_variance.getZ();

        out.reset_counter = 0; // not published
        out.quality = 0; // unused

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
