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
    this->model = _model;
    this->link = model->GetLink("link_base");

    if (!this->link)
    {
      gzerr << "[ERROR] Link 'link_base' not found in model! Plugin will not apply force.\n";
      return;
    }

    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->ros_node = std::make_shared<rclcpp::Node>("caudal_force_plugin");

    this->velocity_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
        "/caudal_velocity", 10,
        std::bind(&CaudalForcePlugin::OnVelocityMsg, this, std::placeholders::_1));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CaudalForcePlugin::OnUpdate, this));

    gzmsg << "[INFO] CaudalForcePlugin loaded! Applying COM-centered force with yaw counter-torque on " << this->link->GetName() << "\n";
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

    double k = 1.0;
    double force_x = k * std::abs(this->caudal_velocity) * this->caudal_velocity;
    ignition::math::Vector3d force(force_x, 0, 0);

    ignition::math::Vector3d com = this->link->GetInertial()->Pose().Pos();
    this->link->AddForceAtRelativePosition(force, com);

    // Apply counter-torque to cancel yaw (z-axis rotation)
    double kp = 10.0; // Proportional gain for yaw damping (adjust as needed)
    ignition::math::Vector3d angular_vel = this->link->WorldAngularVel();
    ignition::math::Vector3d counter_torque(0, 0, -kp * angular_vel.Z());
    this->link->AddTorque(counter_torque);

    // Constrain linear velocity to X-axis only
    double vx = this->link->WorldLinearVel().X();
    this->link->SetLinearVel(ignition::math::Vector3d(vx, 0, 0));

    gzdbg << "[DEBUG] Applied force: " << force << " at COM: " << com
          << " | Counter-torque: " << counter_torque
          << " | AngularVel: " << angular_vel
          << " | LinearVel: (" << vx << ", 0, 0)"
          << " | Caudal velocity command: " << this->caudal_velocity << "\n";

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

GZ_REGISTER_MODEL_PLUGIN(gazebo::CaudalForcePlugin)
}