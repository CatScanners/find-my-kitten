#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <string>
#include <math.h>

using namespace std::chrono;
using namespace std::chrono_literals;

class SetpointPublisher : public rclcpp::Node
{
  public:
    // PARAMETERS:
    std::string deviation_topic;
    float cam_scale[2]; // how many meters are image height/2 and width/2
    float cam_rotation_x;
    float cam_rotation_y;
    float cam_rotation_z;
    float height;  // drone's height (Down)

    // RUNTIME VARIABLES:
    float position[2]; // drone's horizontal position coords (North, East)
    float setpoint[2]; // North, East
    float heading;     // drone's clockwise orientation along z-axis (radians) [-PI, PI]
    float deviation [2];     // target deviation coords (x, y)

    // Constructor
    SetpointPublisher() : Node("setpoint_publisher")
    {

      // Declare parameters with defaults
      this->declare_parameter("deviation_topic", "target_deviation_topic");
      this->declare_parameter("cam_scale_x", 1.0);
      this->declare_parameter("cam_scale_y", 1.0);
      this->declare_parameter("cam_rotation_x", 0.0);
      this->declare_parameter("cam_rotation_y", 0.0);
      this->declare_parameter("cam_rotation_z", 0.0);

      // Get parameter values
      this->get_parameter("deviation_topic", deviation_topic);
      this->get_parameter("cam_scale_x", cam_scale[0]);
      this->get_parameter("cam_scale_y", cam_scale[1]);
      this->get_parameter("cam_rotation_x", cam_rotation_x);
      this->get_parameter("cam_rotation_y", cam_rotation_y);
      this->get_parameter("cam_rotation_z", cam_rotation_z);

      // Create subscribers
      deviation_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        deviation_topic, 
        10, 
        std::bind(&SetpointPublisher::deviation_callback, this, std::placeholders::_1));

      location_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", 
        10,
        std::bind(&SetpointPublisher::location_callback, this, std::placeholders::_1));

      // Create publishers
      offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
      
      trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

      // Create timer for setpoint updating and offboard control heartbeat
      timer_ = this->create_wall_timer(
        100ms, std::bind(&SetpointPublisher::timer_callback, this));
    }
  private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr deviation_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr location_subscriber_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<uint64_t> timestamp_; //synced timestamp ?

    // CALLBACKS:
    // Callback for deviation topic subscription
    void deviation_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      for (int i = 0; i < 2; i++){
        deviation[i] = msg->data[i];
      }

      RCLCPP_INFO(this->get_logger(), "Received deviation -> X: %.2f, Y: %.2f", deviation[0], deviation[1]);

      // update setpoint
      update_setpoint();
    }

    // Callback for vehicle location subscription
    void location_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
      position[0] = msg->x;
      position[1] = msg->y;
      heading = msg->heading;
    }

    // Timer callback for sending trajectory setpoints and offboard control heartbeat
    void timer_callback()
    {
      publish_offboard_control_mode();
      publish_trajectory_setpoint();

      RCLCPP_INFO(this->get_logger(), "Trajectory setpoint sent.");
    }

    //HELPER FUNCTIONS:
    void update_setpoint()
    {
      float scaled_deviation[2];
      float setpoint[2];
      
      // scale deviation
      for (int i = 0; i < 2; i++){
        scaled_deviation[i] = cam_scale[i] * deviation[i];
      }

      // apply rotation [x*cos(a) - y*sin(a); x*sin(a) + y*cos(a)]
      // reverse order as setpoint is [North, East]
      setpoint[1] = scaled_deviation[0] * std::cos(heading) - scaled_deviation[1] * std::sin(heading);
      setpoint[0] = scaled_deviation[0] * std::sin(heading) + scaled_deviation[1] * std::cos(heading);

      // add drone position to the setpoint as an offset 
      for (int i = 0; i < 2; i++){
        setpoint[i] = position[i];
      }

      RCLCPP_INFO(this->get_logger(), "Setpoint set to -> North: %.2f, East: %.2f", setpoint[0], setpoint[1]);
    }


    void publish_offboard_control_mode()
    {
      px4_msgs::msg::OffboardControlMode msg{};
      msg.position = true;
      msg.velocity = false;
      msg.acceleration = false;
      msg.attitude = false;
      msg.body_rate = false;
      msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
      px4_msgs::msg::TrajectorySetpoint msg{};
      msg.position = {setpoint[0], setpoint[1], height};
      msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      trajectory_setpoint_publisher_->publish(msg);
    }
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointPublisher>());
  rclcpp::shutdown();
  return 0;
}