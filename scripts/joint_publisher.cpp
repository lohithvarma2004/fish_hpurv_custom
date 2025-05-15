#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher()
  : Node("joint_publisher"), count_(0)
  {
    // Publish to the trajectory controller topic (ensure your controller is listening on this topic)
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);
    
    // Timer to publish messages at 10 Hz
    timer_ = this->create_wall_timer(
      100ms, std::bind(&JointPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = trajectory_msgs::msg::JointTrajectory();
    
    // Set joint names as in your URDF under the <ros2_control> block
    message.joint_names = {"caudal_joint", "joint1", "joint2"};
    
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();

    // Define maximum amplitudes (in radians):
    // Tail: ±30° (≈0.5236 rad), fins: ±45° (≈0.7854 rad)
    const double tail_amplitude = 0.5236;
    const double pec1_fin_amplitude = 0.7854;
    const double pec2_fin_amplitude = 0.5236;

    // Compute oscillatory positions with a chosen frequency
    double tail_position = tail_amplitude * std::sin(count_ * 0.05);
    double pec1_fin_position  = pec1_fin_amplitude  * std::sin(count_ * 0.05);
    double pec2_fin_position  = pec2_fin_amplitude  * std::sin(count_ * 0.05);

    // For the fins, we make one go positive and the other negative so that they move oppositely.
    point.positions = {tail_position, pec1_fin_position, pec2_fin_position};

    // Optionally, set velocities if required by your controller.
    point.velocities = {0.0, 0.0, 0.0};
    // Define how long this trajectory point should take (here, 1 second)
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);

    message.points.push_back(point);
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(),
                "Publishing positions: tail: %f, joint1: %f, joint2: %f",
                tail_position, pec1_fin_position, pec2_fin_position);

    ++count_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPublisher>());
  rclcpp::shutdown();
  return 0;
}
