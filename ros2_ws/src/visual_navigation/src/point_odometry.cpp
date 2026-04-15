#include <cmath>
#include <memory>
#include <optional>
#include <px4_msgs/msg/detail/vehicle_odometry__struct.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <string>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detail/detection2_d__struct.hpp>
#include <vision_msgs/msg/detail/detection2_d_array__struct.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "Quaternion.hpp"
#include "drone.hpp"
#include "inputPoint.hpp"

// Algorithm (drone.cpp) uses camera-RUB body + ENU (Z-up) world; PX4 uses FRD body + NED world.
// Both quaternions are (w,x,y,z) and rotate body -> world.
// Q_NED_ENU: NED<->ENU world basis change (180° about (1,1,0)/sqrt(2), self-inverse).
// Q_CAM_BODY: down-facing camera mount, cam-RUB<->body-FRD (also 180° about (1,1,0)/sqrt(2), self-inverse).
// Composition: q_alg = Q_NED_ENU * q_px4 * Q_CAM_BODY, and the reverse uses the same formula.
static const Quaternion Q_NED_ENU  = {0.0f, std::sqrt(0.5f), std::sqrt(0.5f), 0.0f};
static const Quaternion Q_CAM_BODY = {0.0f, std::sqrt(0.5f), std::sqrt(0.5f), 0.0f};

#define LOG(severity, fmt, ...) RCLCPP_##severity(this->get_logger(), fmt, ##__VA_ARGS__)

using std::placeholders::_1;

class PointOdometry : public rclcpp::Node {
public:
  PointOdometry() : Node("point_odometry") {
    detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "detections", 10, std::bind(&PointOdometry::detection_callback, this, _1));
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&PointOdometry::odometry_callback, this, _1));

    // publisher that publishes point odometry messages to point_odometry
    publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", 10);

    LOG(INFO, "Initialized");
  }

private:
  // PX4 position is NED (x=N, y=E, z=D). Algorithm world is ENU (x=E, y=N, z=U).
  // Assumes incoming odometry uses pose_frame = POSE_FRAME_NED.
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    DroneState out{};
    out.loc.x = msg->position[1];  // Y
    out.loc.y = msg->position[0];  // X
    out.loc.z = -msg->position[2]; // Z

    Quaternion q;
    q.w = msg->q[0];
    q.x = msg->q[1];
    q.y = msg->q[2];
    q.z = msg->q[3];

    out.rot = quat_px4_to_point_odom(q);

    if (!drone_initialized_) {
      drone_ = drone(out);
      drone_initialized_ = true;
      LOG(INFO, "Initialized drone struct with positions: %f, %f, %f\n", out.loc.x, out.loc.y,
          out.loc.z);
    }
    internal_state_ = out;
  };

  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    // drone not initialized with its state, can't run
    if (!drone_initialized_)
      return;

    if (msg->detections.empty())
      return;

    const std::vector<inputPoint> objects = objects_from_detections(msg->detections);

    const std::optional<DroneState> state_opt =
        drone_.process_frames(objects, internal_state_);
    if (!state_opt)
      return;
    const DroneState state = state_opt.value();
    px4_msgs::msg::VehicleOdometry out = odom_from_drone_state(state);
    out.timestamp = now().nanoseconds() / 1'000;
    out.timestamp_sample = static_cast<uint64_t>(msg->header.stamp.sec) * 1'000'000 +
                           static_cast<uint64_t>(msg->header.stamp.nanosec) / 1'000;

    LOG(INFO, "Internal state: %f, %f, %f\n", internal_state_.loc.x, internal_state_.loc.y,
        internal_state_.loc.z);
    LOG(INFO, "Publishing position: %f, %f, %f\n", out.position[0], out.position[1],
        out.position[2]);
    publisher_->publish(out);
  }

  std::vector<inputPoint>
  objects_from_detections(const std::vector<vision_msgs::msg::Detection2D> &detections) {
    std::vector<inputPoint> objects;

    for (const vision_msgs::msg::Detection2D &detection : detections) {
      if (detection.id == "")
        continue;

      // trust it will be parseable :D
      int id = std::stoi(detection.id);
      float x = static_cast<float>(detection.bbox.center.position.x);
      float y = static_cast<float>(detection.bbox.center.position.y);
      inputPoint input = convertToUsableForm(1640, 1232, 170, id, x, y, true);
      objects.push_back(input);
    }

    return objects;
  }

  // Algorithm world is ENU (x=E, y=N, z=U); PX4 position is NED (x=N, y=E, z=D).
  px4_msgs::msg::VehicleOdometry odom_from_drone_state(const DroneState &state) {
    px4_msgs::msg::VehicleOdometry out{};
    out.pose_frame = out.POSE_FRAME_FRD;

    float x = state.loc.y;
    float y = state.loc.x;
    float z = -state.loc.z;
    out.position = {x, y, z};

    const Quaternion rotated = quat_point_odom_to_px4(state.rot);

    out.q = {static_cast<float>(rotated.w), static_cast<float>(rotated.x),
             static_cast<float>(rotated.y), static_cast<float>(rotated.z)};

    out.velocity_frame = out.VELOCITY_FRAME_UNKNOWN;
    out.velocity = {NAN, NAN, NAN};
    out.angular_velocity = {NAN, NAN, NAN};

    // just set variances :D
    out.position_variance = {1, 1, 1};
    out.orientation_variance = {0.10, 0.10, 0.20};
    out.reset_counter = 0;

    return out;
  }

  Quaternion quat_px4_to_point_odom(Quaternion q_px4) {
    return Q_NED_ENU * q_px4 * Q_CAM_BODY;
  }

  Quaternion quat_point_odom_to_px4(Quaternion q_alg) {
    return Q_NED_ENU * q_alg * Q_CAM_BODY;
  }

  DroneState internal_state_{};
  drone drone_{DroneState{}};
  bool drone_initialized_ = false;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointOdometry>());
  rclcpp::shutdown();
  return 0;
}
