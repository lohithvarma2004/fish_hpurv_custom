#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
class GliderPitchBuoyancyPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
  {
    this->model = _model;
    this->link = model->GetLink("link_base");

    if (!this->link)
    {
      gzerr << "[ERROR] Link 'link_base' not found! Plugin will not apply forces.\n";
      return;
    }

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    this->ros_node = std::make_shared<rclcpp::Node>("glider_pitch_buoyancy_plugin");

    this->mode_sub = ros_node->create_subscription<std_msgs::msg::String>(
        "/glider_mode", 10,
        std::bind(&GliderPitchBuoyancyPlugin::OnModeMsg, this, std::placeholders::_1));

    this->vertical_force_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
        "/buoyancy_force_z", 10,
        std::bind(&GliderPitchBuoyancyPlugin::OnVerticalForceMsg, this, std::placeholders::_1));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GliderPitchBuoyancyPlugin::OnUpdate, this));

    gzmsg << "[INFO] GliderPitchBuoyancyPlugin loaded. Waiting for /glider_mode command.\n";
  }

  void OnModeMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string mode = msg->data;

    if (mode == "descent" || mode == "ascent")
    {
      glider_mode = mode;
      pitching_started = false;
      pitch_complete = false;
      glide_started = false;
      pitch_start_time = this->model->GetWorld()->SimTime().Double();
      gzmsg << "[INFO] Mode change: " << mode << " at time: " << pitch_start_time << "\n";
    }
    else if (mode == "idle")
    {
      glider_mode = "idle";
      pitching_started = pitch_complete = glide_started = false;
      gzmsg << "[INFO] Mode set to IDLE.\n";
    }
    else
    {
      gzwarn << "[WARN] Unknown mode: " << mode << "\n";
    }
  }

  void OnVerticalForceMsg(const std_msgs::msg::Float64::SharedPtr msg)
  {
    this->vertical_force_z = msg->data;
  }

  void OnUpdate()
  {
    rclcpp::spin_some(this->ros_node);

    if (glider_mode == "idle" || !this->link)
      return;

    double sim_time = this->model->GetWorld()->SimTime().Double();
    ignition::math::Quaterniond rot = this->link->WorldPose().Rot();
    double pitch = rot.Euler().Y();
    ignition::math::Vector3d ang_vel = this->link->WorldAngularVel();
    ignition::math::Vector3d pos = this->link->WorldPose().Pos();

    const double pitch_gain = 1.5;
    const double pitch_tolerance = 0.05;
    const double glide_delay = 15.0;

    double target_pitch = 0.0;

    if (glider_mode == "descent")
      target_pitch = +0.5;
    else if (glider_mode == "ascent")
      target_pitch = -0.5;

    // Apply buoyancy force from /buoyancy_force_z topic
    ignition::math::Vector3d buoyancy_force(-vertical_force_z, 0, 0);
    this->link->AddForce(buoyancy_force);

    // Pitch control
    if (!pitch_complete)
    {
      double pitch_error = target_pitch - pitch;
      if (std::abs(pitch_error) > pitch_tolerance)
      {
        double torque = pitch_gain * pitch_error;
        this->link->AddRelativeTorque(ignition::math::Vector3d(0, -torque, 0));
        pitching_started = true;
      }
      else if (pitching_started)
      {
        this->link->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
        pitch_complete = true;
        pitch_complete_time = sim_time;
        gzmsg << "[INFO] [" << glider_mode << "] Pitch complete at time: " << sim_time << "\n";
      }
    }

    // Maintain pitch
    if (pitch_complete)
    {
      double pitch_error = target_pitch - pitch;
      double corrective_torque = 10.0 * pitch_error;
      this->link->AddRelativeTorque(ignition::math::Vector3d(0, corrective_torque, 0));
    }

    // Glide phase
    if (pitch_complete && !glide_started && sim_time >= pitch_complete_time + glide_delay)
    {
      glide_started = true;
      gzmsg << "[INFO] [" << glider_mode << "] Starting glide at time: " << sim_time << "\n";
    }

    if (glide_started)
    {
      ignition::math::Vector3d vel = this->link->WorldLinearVel();
      gzdbg << "[DEBUG] [" << glider_mode << "] Time: " << sim_time
            << " | pitch: " << pitch
            << " | vel: " << vel
            << " | buoyancy_force_z: " << vertical_force_z << "\n";
    }

    // Constrain lateral velocity
    ignition::math::Vector3d world_vel = this->link->WorldLinearVel();
    ignition::math::Vector3d local_vel = rot.RotateVectorReverse(world_vel);
    local_vel.Y() = 0;
    ignition::math::Vector3d constrained_vel = rot.RotateVector(local_vel);
    this->link->SetLinearVel(constrained_vel);

    // Angular damping
    ignition::math::Vector3d damp_torque(
        -damping_k * ang_vel.X(),
        -damping_k * ang_vel.Y(),
        -damping_k * ang_vel.Z());
    this->link->AddTorque(damp_torque);
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  event::ConnectionPtr updateConnection;

  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vertical_force_sub;

  std::string glider_mode = "idle";
  bool pitching_started = false;
  bool pitch_complete = false;
  bool glide_started = false;

  double vertical_force_z = 0.0;
  double damping_k = 5.0;
  double pitch_start_time = 0.0;
  double pitch_complete_time = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(GliderPitchBuoyancyPlugin)
} // namespace gazebo