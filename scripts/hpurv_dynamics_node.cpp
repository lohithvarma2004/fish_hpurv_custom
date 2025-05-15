#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class HPURVDynamicsNode : public rclcpp::Node
{
public:
  HPURVDynamicsNode() : Node("hpurv_dynamics_node")
  {
    // Initialize model parameters (from MATLAB)
    m_ = 13.0; // Mass in kg
    rho_ = 1021.0; // Density of water
    g_ = 9.81; // Gravity
    a_ = 0.17; b_ = 0.24; L_ = 0.740; // Dimensions
    xG_ = 0.0; yG_ = 0.0; zG_ = 0.0; // CoG
    xB_ = 0.0; yB_ = 0.0; zB_ = 0.0; // CoB
    IxG_ = 0.05; IyG_ = 0.45; IzG_ = 0.44; // MoI about CoG
    Ixy_ = 0.0; Iyz_ = 0.0; Ixz_ = 0.0; // Products of inertia
    Ix_ = IxG_ + m_ * (yG_ * yG_ + zG_ * zG_);
    Iy_ = IyG_ + m_ * (xG_ * xG_ + zG_ * zG_);
    Iz_ = IzG_ + m_ * (xG_ * xG_ + yG_ * yG_);
    x1_ = -0.257; y1_ = 0.0; z1_ = 0.0; // Fin positions
    x2_ = 0.177; y2_ = 0.097; z2_ = 0.0495;
    x3_ = 0.177; y3_ = -0.097; z3_ = 0.0495;
    Cl_ = 0.92; Cd_ = 1.12; // Lift and drag coefficients
    S1_ = 0.024; L_f1_ = 0.2; // Caudal fin
    S2_ = 0.044; L_f2_ = 0.1; // Fin 2
    S3_ = 0.044; L_f3_ = 0.1; // Fin 3
    PF1max_ = 5.0;
    freqmax_ = 2.0;
    dt_ = 0.05; // Time step
    Ux_ = 0.4; Uy_ = 0.4; Uz_ = 0.0; // Free flow velocity

    // Initialize mass matrix (Mrb)
    Mrb_ = Eigen::MatrixXd::Zero(6, 6);
    Mrb_ << m_, 0, 0, 0, m_ * zG_, -m_ * yG_,
            0, m_, 0, -m_ * zG_, 0, m_ * xG_,
            0, 0, m_, m_ * yG_, -m_ * xG_, 0,
            0, -m_ * zG_, m_ * yG_, Ix_, -Ixy_, -Ixz_,
            m_ * zG_, 0, -m_ * xG_, -Ixy_, Iy_, -Iyz_,
            -m_ * yG_, m_ * xG_, 0, -Ixz_, -Iyz_, Iz_;

    // Added mass
    Xudot_ = -1.3; Yvdot_ = -2.3; Zwdot_ = -5.5;
    Kpdot_ = -0.06; Nrdot_ = -2.04; Mqdot_ = -0.86;
    Mad_ = Eigen::MatrixXd::Zero(6, 6);
    Mad_.diagonal() << Xudot_, Yvdot_, Zwdot_, Kpdot_, Mqdot_, Nrdot_;
    M_ = Mrb_ + Mad_; // Total mass

    // Control gains
    Kp_ = 4.0 * Mrb_;
    Kd_ = 4.0 * Mrb_;

    // Initialize state vectors
    nu_ = Eigen::VectorXd::Zero(6);
    eta_ = Eigen::VectorXd::Zero(6);
    eta_(0) = 0.0; eta_(1) = 0.0; eta_(2) = 0.0; // Initial pose
    eta_(3) = 0.0; eta_(4) = 0.0; eta_(5) = 0.0;

    // Fin angles
    b1_ = 30.0 * M_PI / 180.0;
    b2_ = 30.0 * M_PI / 180.0;
    b3_ = 30.0 * M_PI / 180.0;

    // ROS 2 subscriptions and publications
    sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10,
      std::bind(&HPURVDynamicsNode::OnJointTrajectory, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(
      "/hpurv/pose", 10);
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);

    // Timer for dynamics update (20 Hz, dt = 0.05s)
    timer_ = create_wall_timer(
      50ms, std::bind(&HPURVDynamicsNode::TimerCallback, this));

    RCLCPP_INFO(get_logger(), "HPURV Dynamics Node initialized");
  }

private:
  void OnJointTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    // Extract fin angles from JointTrajectory message
    if (!msg->points.empty() && msg->points[0].positions.size() == 3)
    {
      b1_ = msg->points[0].positions[0]; // caudal_joint
      b2_ = msg->points[0].positions[1]; // joint1
      b3_ = msg->points[0].positions[2]; // joint2
    }
  }

  void TimerCallback()
  {
    // Get current time (for trajectory)
    double t = this->now().seconds();

    // Compute Jacobian (J1 and J2)
    double phi = eta_(3), theta = eta_(4), psi = eta_(5);
    Eigen::Matrix3d J1;
    J1 << cos(psi) * cos(theta), -cos(phi) * sin(psi) + cos(psi) * sin(phi) * sin(theta), sin(psi) * sin(phi) + cos(phi) * cos(psi) * sin(theta),
          sin(psi) * cos(theta), cos(psi) * cos(theta) + sin(psi) * sin(phi) * sin(theta), -cos(psi) * sin(phi) + cos(phi) * sin(psi) * sin(theta),
          -sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta);

    Eigen::Matrix3d J2;
    J2 << 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
          0, cos(phi), -sin(phi),
          0, sin(phi) / cos(theta), cos(phi) / cos(theta);

    Eigen::MatrixXd J(6, 6);
    J.setZero();
    J.block<3,3>(0,0) = J1;
    J.block<3,3>(3,3) = J2;

    // Compute desired trajectory (eight-shape profile)
    double fi = 0.01;
    Eigen::VectorXd etad(6), etad_dot(6), etad_ddot(6);
    etad << 4 * sin(2 * fi * t), 4 - 4 * cos(fi * t), 2 - 2 * cos(fi * t), 0, 0, 0;
    etad_dot << 4 * 2 * fi * cos(2 * fi * t), 4 * fi * sin(fi * t), 2 * fi * sin(fi * t), 0, 0, 0;
    etad_ddot << -4 * 4 * fi * fi * sin(2 * fi * t), 4 * fi * fi * cos(fi * t), 2 * fi * fi * cos(fi * t), 0, 0, 0;

    // Update yaw based on Line-of-Sight (LoS)
    etad(5) = std::atan2(etad_dot(1), etad_dot(0));

    // Compute error
    Eigen::VectorXd e1 = etad - eta_;
    Eigen::VectorXd e1dot = etad_dot - J * nu_;

    // Compute Coriolis and centripetal matrix (crb)
    double u = nu_(0), v = nu_(1), w = nu_(2), p = nu_(3), q = nu_(4), r = nu_(5);
    Eigen::MatrixXd crb(6, 6);
    crb << 0, 0, 0, m_ * (yG_ * q + zG_ * r), -m_ * (xG_ * q - w), -m_ * (xG_ * r + v),
           0, 0, 0, -m_ * (yG_ * p + w), m_ * (zG_ * r + xG_ * p), -m_ * (yG_ * r - u),
           0, 0, 0, -m_ * (zG_ * p - v), -m_ * (xG_ * q + u), m_ * (zG_ * p + yG_ * q),
           -m_ * (yG_ * q + zG_ * r), m_ * (yG_ * p + w), m_ * (zG_ * p - v), 0, -q * Iyz_ - p * Ixz_ + r * Iz_, r * Iyz_ + p * Ixy_ - q * Iy_,
           m_ * (xG_ * q - w), -m_ * (zG_ * r + xG_ * p), m_ * (xG_ * q + u), q * Iyz_ + p * Ixz_ - r * Iz_, 0, -r * Ixz_ - q * Ixy_ + p * Ix_,
           m_ * (xG_ * r + v), m_ * (yG_ * r - u), -m_ * (zG_ * p + yG_ * q), -r * Iyz_ - p * Ixy_ + q * Iy_, r * Ixz_ + q * Ixy_ - p * Ix_, 0;

    // Added mass Coriolis (cad)
    Eigen::MatrixXd cad(6, 6);
    cad << 0, 0, 0, 0, -Zwdot_ * w, Yvdot_ * v,
           0, 0, 0, Zwdot_ * w, 0, -Xudot_ * u,
           0, 0, 0, -Yvdot_ * v, Xudot_ * u, 0,
           0, -Zwdot_ * w, Yvdot_ * v, 0, -Nrdot_ * r, Mqdot_ * q,
           Zwdot_ * w, 0, -Xudot_ * u, Nrdot_ * r, 0, -Kpdot_ * p,
           -Yvdot_ * v, Xudot_ * u, 0, -Mqdot_ * q, Kpdot_ * p, 0;

    Eigen::MatrixXd cn = crb + cad;
    Eigen::VectorXd cm = cn * nu_;

    // Damping matrix
    Eigen::MatrixXd dn(6, 6);
    dn << 0.97 * std::abs(u), 0, 0, 0, 0, 0,
          0, 5.08 * std::abs(v), 0, 0, 0, 0,
          0, 0, 3.38 * std::abs(w), 0, 0, 0,
          0, 0, 0, 0.004 * std::abs(p), 0, 0,
          0, 0, 0, 0, 0.004 * std::abs(q), 0,
          0, 0, 0, 0, 0, 0.05 * std::abs(r);

    Eigen::VectorXd dm = dn * nu_;

    // Control law (PD control)
    Eigen::VectorXd tau = J.transpose() * (Kd_ * e1dot + Kp_ * e1);

    // Compute fin forces (B matrix)
    Eigen::MatrixXd B(6, 6);
    B << sin(b1_), cos(b1_), sin(b2_), cos(b2_), sin(b3_), cos(b3_),
         cos(b1_), -sin(b1_), 0, 0, 0, 0,
         0, 0, cos(b2_), -sin(b2_), cos(b3_), -sin(b3_),
         -z1_ * cos(b1_), z1_ * sin(b1_), y2_ * cos(b2_), -y2_ * sin(b2_), y3_ * cos(b3_), -y3_ * sin(b3_),
         z1_ * sin(b1_), z1_ * cos(b1_), (z2_ * sin(b2_) - x2_ * cos(b2_)), (z2_ * cos(b2_) + x2_ * sin(b2_)), (z3_ * sin(b3_) - x3_ * cos(b3_)), (z3_ * cos(b3_) + x3_ * sin(b3_)),
         (-x1_ * cos(b1_) - y1_ * sin(b1_)), -(-x1_ * sin(b1_) + y1_ * cos(b1_)), -y2_ * sin(b2_), -y2_ * cos(b2_), -y3_ * sin(b3_), -y3_ * cos(b3_);

    Eigen::VectorXd f = B.inverse() * tau;

    // Paddle forces
    double PF1 = std::sqrt(f(0) * f(0) + f(1) * f(1));
    double PF2 = std::sqrt(f(2) * f(2) + f(3) * f(3));
    double PF3 = std::sqrt(f(4) * f(4) + f(5) * f(5));

    // Body force angles
    double BFA1 = std::atan2(f(1), 0.0001 + f(0));
    double BFA2 = std::atan2(f(3), 0.0001 + f(2));
    double BFA3 = std::atan2(f(5), 0.0001 + f(4));

    // Body forces
    double Fx1 = PF1 * std::sin(BFA1 + b1_);
    double Fy1 = PF1 * std::cos(BFA1 + b1_);
    double Fx2 = PF2 * std::sin(BFA2 + b2_);
    double Fz2 = PF2 * std::cos(BFA2 + b2_);
    double Fx3 = PF3 * std::sin(BFA3 + b3_);
    double Fz3 = PF3 * std::cos(BFA3 + b3_);

    // Update fin angles
    b1_ = std::atan2(Fx1, 0.0001 + Fy1) - BFA1;
    b2_ = std::atan2(Fx2, 0.0001 + Fz2) - BFA2;
    b3_ = std::atan2(Fx3, 0.0001 + Fz3) - BFA3;

    // Compute total force and torque
    Eigen::VectorXd force_torque(6);
    force_torque << Fx1 + Fx2 + Fx3, Fy1, Fz2 + Fz3,
                    -z1_ * Fy1 + y2_ * Fz2 + y3_ * Fz3,
                    z1_ * Fx1 + (z2_ * Fx2 - x2_ * Fz2) + (z3_ * Fx3 - x3_ * Fz3),
                    (-x1_ * Fy1 - y1_ * Fx1) - y2_ * Fx2 - y3_ * Fx3;

    // Compute acceleration (M * nu_dot = tau - cm - dm)
    Eigen::VectorXd nu_dot = M_.inverse() * (force_torque - cm - dm);

    // Integrate velocity (nu = nu + nu_dot * dt)
    nu_ += nu_dot * dt_;

    // Integrate position (eta = eta + J * nu * dt)
    eta_ += J * nu_ * dt_;

    // Publish joint states
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name = {"caudal_joint", "joint1", "joint2"};
    joint_state.position = {b1_, b2_, b3_};
    joint_state_pub_->publish(joint_state);

    // Publish joint trajectory commands
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = this->now();
    trajectory_msg.joint_names = {"caudal_joint", "joint1", "joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {b1_, b2_, b3_};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0); // Smooth control
    trajectory_msg.points.push_back(point);
    trajectory_pub_->publish(trajectory_msg);

    // Publish pose
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = eta_(0);
    pose_msg.position.y = eta_(1);
    pose_msg.position.z = eta_(2);
    double roll = eta_(3), pitch = eta_(4), yaw = eta_(5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    pose_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    pose_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    pose_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    pose_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    pose_pub_->publish(pose_msg);
  }

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  // Model parameters
  double m_, rho_, g_, a_, b_, L_;
  double xG_, yG_, zG_, xB_, yB_, zB_;
  double IxG_, IyG_, IzG_, Ixy_, Iyz_, Ixz_, Ix_, Iy_, Iz_;
  double x1_, y1_, z1_, x2_, y2_, z2_, x3_, y3_, z3_;
  double Cl_, Cd_, S1_, L_f1_, S2_, L_f2_, S3_, L_f3_;
  double PF1max_, freqmax_, dt_;
  double Xudot_, Yvdot_, Zwdot_, Kpdot_, Nrdot_, Mqdot_;
  double Ux_, Uy_, Uz_;
  double b1_, b2_, b3_;

  // Matrices and vectors
  Eigen::MatrixXd Mrb_, Mad_, M_, Kp_, Kd_;
  Eigen::VectorXd nu_, eta_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HPURVDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}