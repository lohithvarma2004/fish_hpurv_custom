#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
class CaudalForcePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
  {
    // Store model and link
    this->model = _model;
    this->link = model->GetLink("link_base");

    if (!this->link)
    {
      gzerr << "[ERROR] Link 'link_base' not found in model! Plugin will not apply force.\n";
      return;
    }

    // Initialize ROS 2 node
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->ros_node = std::make_shared<rclcpp::Node>("caudal_force_plugin");

    // Subscribe to caudal velocity
    this->velocity_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
        "/caudal_velocity", 10,
        std::bind(&CaudalForcePlugin::OnVelocityMsg, this, std::placeholders::_1));

    // Connect to Gazebo update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CaudalForcePlugin::OnUpdate, this));

    gzmsg << "[INFO] CaudalForcePlugin loaded! Applying force on " << this->link->GetName() << "\n";
  }

  void OnVelocityMsg(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->caudal_velocity = msg->data;
  }

  void OnUpdate()
  {
    if (!this->link)
    {
      gzerr << "[ERROR] Link is null. Force cannot be applied!\n";
      return;
    }

    // Compute force: F_x = k * |v| * v
    double k = 5.0; // Thrust coefficient (N·s/rad²)
    double force_x = k * std::abs(this->caudal_velocity) * this->caudal_velocity;
    ignition::math::Vector3d force(force_x, 0, 0); // Forward force in link_base frame

    // Apply force to link_base
    this->link->AddRelativeForce(force);

    gzdbg << "[DEBUG] Applied force: " << force << " on " << this->link->GetName()
          << " for caudal velocity: " << this->caudal_velocity << "\n";

    // Spin ROS node to process callbacks
    rclcpp::spin_some(this->ros_node);
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  event::ConnectionPtr updateConnection;
  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;
  double caudal_velocity = 0.0;
};

// Register plugin
GZ_REGISTER_MODEL_PLUGIN(gazebo::CaudalForcePlugin)
}