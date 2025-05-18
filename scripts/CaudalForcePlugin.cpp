#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

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

    // Initialize last message time
    last_msg_time = this->ros_node->get_clock()->now();

    gzmsg << "[INFO] CaudalForcePlugin loaded! Applying local x-axis force with rotation damping on " 
          << this->link->GetName() << "\n";
  }

  void OnVelocityMsg(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->caudal_velocity = msg->data;
    last_msg_time = this->ros_node->get_clock()->now();
  }

  void OnUpdate()
  {
    if (!this->link)
    {
      gzerr << "[ERROR] Link is null. Force cannot be applied!\n";
      return;
    }

    // Check for message timeout (0.1 seconds)
    auto now = this->ros_node->get_clock()->now();
    if ((now - last_msg_time).nanoseconds() * 1e-9 > 0.1)
    {
      caudal_velocity = 0.0;
      gzdbg << "[DEBUG] Timeout: No /caudal_velocity messages for 0.1s, reset caudal_velocity to 0\n";
    }

    double k = 1.0;
    double force_x = k * std::abs(this->caudal_velocity) * caudal_velocity;
    ignition::math::Vector3d force_local(force_x, 0, 0);

    ignition::math::Vector3d com = this->link->GetInertial()->Pose().Pos();
    ignition::math::Quaterniond rot = this->link->WorldPose().Rot();
    ignition::math::Vector3d force_world = rot.RotateVector(force_local);
    this->link->AddForceAtRelativePosition(force_world, com);

    // Apply counter-torque to damp rotation (yaw, pitch, roll)
    double kp = 10.0; // Proportional gain for damping
    ignition::math::Vector3d angular_vel = this->link->WorldAngularVel();
    ignition::math::Vector3d counter_torque(
        -kp * angular_vel.X(), // Damp roll
        -kp * angular_vel.Y(), // Damp pitch
        -kp * angular_vel.Z()  // Damp yaw
    );
    this->link->AddTorque(counter_torque);

    // Constrain linear velocity to local x-axis
    ignition::math::Vector3d world_vel = this->link->WorldLinearVel();
    ignition::math::Vector3d local_vel = rot.RotateVectorReverse(world_vel);
    local_vel.Y() = 0; // No sideways motion
    local_vel.Z() = 0; // No vertical motion
    ignition::math::Vector3d constrained_vel = rot.RotateVector(local_vel);
    this->link->SetLinearVel(constrained_vel);

    gzdbg << "[DEBUG] Time: " << now.nanoseconds() * 1e-9
          << " | Applied force (local): " << force_local
          << " | Applied force (world): " << force_world
          << " at COM: " << com
          << " | Counter-torque: " << counter_torque
          << " | AngularVel: " << angular_vel
          << " | LinearVel (world): " << world_vel
          << " | LinearVel (local): " << local_vel
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
  rclcpp::Time last_msg_time;
};

GZ_REGISTER_MODEL_PLUGIN(gazebo::CaudalForcePlugin)
}