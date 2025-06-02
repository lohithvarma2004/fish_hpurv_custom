#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class SimpleBuoyancyNode : public rclcpp::Node
{
public:
  SimpleBuoyancyNode() : Node("simple_buoyancy_node")
  {
    // Parameters (tune these!)
    this->declare_parameter("water_density", 1025.0);       // kg/m^3
    this->declare_parameter("gravity", 9.81);               // m/s^2
    this->declare_parameter("vehicle_volume", 0.02);        // m^3 (displaced volume)
    this->declare_parameter("vehicle_mass", 15.0);          // kg
    this->declare_parameter("target_depth", -5.0);          // m (negative down)
    this->declare_parameter("kp_depth", 10.0);              // proportional gain
    this->declare_parameter("kd_depth", 5.0);               // derivative gain

    this->get_parameter("water_density", water_density_);
    this->get_parameter("gravity", gravity_);
    this->get_parameter("vehicle_volume", volume_);
    this->get_parameter("vehicle_mass", mass_);
    this->get_parameter("target_depth", target_depth_);
    this->get_parameter("kp_depth", kp_depth_);
    this->get_parameter("kd_depth", kd_depth_);

    // Subscribers to vehicle pose and velocity to get depth and vertical velocity
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "vehicle/pose", 10, std::bind(&SimpleBuoyancyNode::poseCallback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "vehicle/twist", 10, std::bind(&SimpleBuoyancyNode::twistCallback, this, std::placeholders::_1));

    buoyancy_pub_ = this->create_publisher<std_msgs::msg::Float64>("buoyancy_force_z", 10);

    // Timer: publish buoyancy force at 100 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&SimpleBuoyancyNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "SimpleBuoyancyNode started");
  }

private:
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    current_depth_ = msg->position.z;  // z is vertical, down is negative
  }

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vertical_velocity_ = msg->linear.z;
  }

  void controlLoop()
  {
    // Buoyancy force: F_b = rho * g * V
    double buoyancy_force = water_density_ * gravity_ * volume_;

    // Weight force: F_w = m * g
    double weight_force = mass_ * gravity_;

    // Net buoyancy force (positive upwards)
    double net_buoyancy = buoyancy_force - weight_force;

    // Depth error control (simple PD controller)
    double depth_error = target_depth_ - current_depth_;
    double depth_correction = kp_depth_ * depth_error - kd_depth_ * vertical_velocity_;

    // Final vertical force = net buoyancy + depth correction
    double total_buoyancy_force = net_buoyancy + depth_correction;

    std_msgs::msg::Float64 msg;
    msg.data = total_buoyancy_force;
    buoyancy_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Depth=%.3f, Vel=%.3f, Buoyancy=%.3f",
                 current_depth_, vertical_velocity_, total_buoyancy_force);
  }

  // Parameters
  double water_density_, gravity_, volume_, mass_, target_depth_;
  double kp_depth_, kd_depth_;

  // State
  double current_depth_ = 0.0;
  double vertical_velocity_ = 0.0;

  // ROS2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr buoyancy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleBuoyancyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
