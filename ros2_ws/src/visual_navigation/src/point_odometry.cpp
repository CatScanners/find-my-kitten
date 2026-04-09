#include <cmath>
#include <memory>
#include <px4_msgs/msg/detail/vehicle_odometry__struct.hpp>
#include <string>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <vision_msgs/msg/detail/detection2_d__struct.hpp>
#include <vision_msgs/msg/detail/detection2_d_array__struct.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#define LOG(severity, fmt, ...) RCLCPP_##severity(this->get_logger(), fmt, ##__VA_ARGS__)

using std::placeholders::_1;

// #include "lassi koodi"

struct TrackedObject {
  int id;
  float x;
  float y;
};

struct Quaternion {
  float w, x, y, z;
};

struct DroneState {
  float x, y, z;
  Quaternion q;
};

class PointOdometry : public rclcpp::Node {
public:
  PointOdometry() : Node("point_odometry") {
    // callback lamba function
    // tämän sisään tulee Lassin koodi, msg tyypit täytyy vielä muuttaa oikeiksi

    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "detections", 10, std::bind(&PointOdometry::detection_callback, this, _1));
    // publisher that publishes point odometry messages to point_odometry
    publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("point_odometry", 10);

    LOG(INFO, "Initialized");
  }

private:
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (msg->detections.empty())
      return;

    const std::vector<TrackedObject> objects = objects_from_detections(msg->detections);

    DroneState state{}; // = lassin_kood(objects);

    px4_msgs::msg::VehicleOdometry out = odom_from_drone_state(state);
    out.timestamp = now().nanoseconds() / 1'000;
    out.timestamp_sample = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000 +
                           static_cast<uint64_t>(msg->header.stamp.nanosec) / 1'000;

    publisher_->publish(out);
  }

  std::vector<TrackedObject>
  objects_from_detections(const std::vector<vision_msgs::msg::Detection2D> &detections) {
    std::vector<TrackedObject> objects;

    for (const vision_msgs::msg::Detection2D &detection : detections) {
      if (detection.id == "")
        continue;

      TrackedObject obj{};
      // trust it will be parseable :D
      obj.id = std::stoi(detection.id);
      obj.x = static_cast<float>(detection.bbox.center.position.x);
      obj.y = static_cast<float>(detection.bbox.center.position.y);
      objects.push_back(std::move(obj));
    }

    return objects;
  }

  // RFU coordinates
  // x = Right
  // y = Forward
  // z = Up
  // Need FRD
  // x = Forward
  // y = Right
  // z = Down
  px4_msgs::msg::VehicleOdometry odom_from_drone_state(DroneState state) {
    px4_msgs::msg::VehicleOdometry out{};
    out.pose_frame = out.POSE_FRAME_FRD;

    float x = state.y;
    float y = state.x;
    float z = -state.z;
    out.position = {x, y, z};

    // These rotations are probably wrong :D

    // Change quaternion basis from RFU to FRD:
    // [x_frd, y_frd, z_frd] = [y_rfu, x_rfu, -z_rfu]
    tf2::Quaternion attitude(state.q.x, state.q.y, state.q.z, state.q.w);
    const double sqrt_half = std::sqrt(0.5);
    tf2::Quaternion basis_change;
    basis_change.setValue(sqrt_half, sqrt_half, 0.0, 0.0);
    attitude = basis_change * attitude * basis_change.inverse();

    // Fixed camera mounting offset: convert down-facing camera attitude to
    // vehicle body attitude (identity = drone forward/level).
    tf2::Quaternion camera_mount_to_body;
    camera_mount_to_body.setRPY(0.0, M_PI_2, 0.0);
    attitude = attitude * camera_mount_to_body;
    attitude.normalize();

    out.q = {static_cast<float>(attitude.getW()), static_cast<float>(attitude.getX()),
             static_cast<float>(attitude.getY()), static_cast<float>(attitude.getZ())};

    out.velocity_frame = out.VELOCITY_FRAME_UNKNOWN;
    out.velocity = {NAN, NAN, NAN};
    out.angular_velocity = {NAN, NAN, NAN};

    // just set variances :D
    out.position_variance = {1, 1, 1};
    out.orientation_variance = {0.10, 0.10, 0.20};
    out.reset_counter = 0;

    return out;
  }

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointOdometry>());
  rclcpp::shutdown();
  return 0;
}
