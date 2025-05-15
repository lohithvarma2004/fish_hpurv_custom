#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class ForceDynamicNode : public rclcpp::Node
{
public:
    ForceDynamicNode() : Node("force_dynamic_node")
    {
        // Set use_sim_time
        set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Initialize model parameters (aligned with URDF)
        m_ = 12.6; // Total mass
        rho_ = 1000.0; // Density of water
        g_ = 9.81; // Gravity
        a_ = 0.17; b_ = 0.24; L_ = 0.740; // Dimensions
        xG_ = 0.0325815; yG_ = 0.0493006; zG_ = -0.0204279; // CoG
        xB_ = 0.0325815; yB_ = 0.0493006; zB_ = 0.02; // CoB
        IxG_ = 0.01; IyG_ = 0.01; IzG_ = 0.01; // MoI
        Ixy_ = 0.0; Iyz_ = 0.0; Ixz_ = 0.0;
        Ix_ = IxG_ + m_ * (yG_ * yG_ + zG_ * zG_);
        Iy_ = IyG_ + m_ * (xG_ * xG_ + zG_ * zG_);
        Iz_ = IzG_ + m_ * (xG_ * xG_ + yG_ * yG_);

        // Fin positions (aligned with URDF)
        x1_ = -0.425323; y1_ = 0.0; z1_ = -0.06; // caudal_joint
        x2_ = 0.05; y2_ = -0.18; z2_ = -0.035; // joint1
        x3_ = 0.05; y3_ = 0.08; z3_ = -0.035; // joint2

        Cl_ = 0.92; Cd_ = 1.12;
        S1_ = 0.024; L_f1_ = 0.2;
        S2_ = 0.044; L_f2_ = 0.1;
        S3_ = 0.044; L_f3_ = 0.1;
        PF1max_ = 5.0;
        freqmax_ = 2.0;
        dt_ = 0.033; // Match controller update rate (30 Hz)
        Ux_ = 0.4; Uy_ = 0.4; Uz_ = 0.0;

        // Initialize state vectors
        nu_ = Eigen::VectorXd::Zero(6);
        eta_ = Eigen::VectorXd::Zero(6);
        eta_(0) = 0.0; eta_(1) = 0.0; eta_(2) = 0.0;
        eta_(3) = 0.0; eta_(4) = 0.0; eta_(5) = 0.0;

        // Fin angles
        b1_ = 0.0; b2_ = 0.0; b3_ = 0.0;

        // ROS 2 subscriptions and publications
        sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10,
            std::bind(&ForceDynamicNode::OnJointTrajectory, this, std::placeholders::_1));

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(
            "/hpurv/pose", 10);
        trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        velocity_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/caudal_velocity", 10);

        // Timer for dynamics update (30 Hz to match controller)
        timer_ = create_wall_timer(
            33ms, std::bind(&ForceDynamicNode::TimerCallback, this));

        RCLCPP_INFO(get_logger(), "Force Dynamic Node initialized");
    }

private:
    void OnJointTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (!msg->points.empty() && msg->points[0].positions.size() == 3)
        {
            b1_ = std::max(-0.523599, std::min(0.523599, msg->points[0].positions[0]));
            b2_ = std::max(-0.785398, std::min(0.785398, msg->points[0].positions[1]));
            b3_ = std::max(-0.785398, std::min(0.785398, msg->points[0].positions[2]));
            RCLCPP_INFO(get_logger(), "Received trajectory: b1=%f, b2=%f, b3=%f", b1_, b2_, b3_);
        }
    }

    void TimerCallback()
    {
        // Use simulation time
        double t = get_clock()->now().nanoseconds() * 1e-9; // Convert to seconds

        // Sinusoidal flapping trajectory
        double fi = 1.0; // 1 Hz for smooth motion
        b1_ = 0.5 * sin(2 * M_PI * fi * t); // Caudal fin
        b2_ = 0.5 * sin(2 * M_PI * fi * t + M_PI / 2); // Pectoral left
        b3_ = 0.5 * sin(2 * M_PI * fi * t + M_PI / 2); // Pectoral right
        b1_ = std::max(-0.523599, std::min(0.523599, b1_));
        b2_ = std::max(-0.785398, std::min(0.785398, b2_));
        b3_ = std::max(-0.785398, std::min(0.785398, b3_));

        // Compute velocities
        double v1 = 0.5 * 2 * M_PI * fi * cos(2 * M_PI * fi * t);
        double v2 = 0.5 * 2 * M_PI * fi * cos(2 * M_PI * fi * t + M_PI / 2);
        double v3 = 0.5 * 2 * M_PI * fi * cos(2 * M_PI * fi * t + M_PI / 2);

        // Publish caudal velocity
        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = v1;
        velocity_pub_->publish(velocity_msg);

        // Approximate pose update (for logging, Gazebo handles actual physics)
        double k = 5.0; // Thrust coefficient (N·s/rad²)
        double force_x = k * std::abs(v1) * v1;
        double acc_x = force_x / m_; // Acceleration (m/s²)
        nu_(0) += acc_x * dt_; // Update velocity (m/s)
        eta_(0) += nu_(0) * dt_; // Update position (m)

        // Publish joint states
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.name = {"caudal_joint", "joint1", "joint2"};
        joint_state.position = {b1_, b2_, b3_};
        joint_state.velocity = {v1, v2, v3};
        joint_state_pub_->publish(joint_state);

        // Publish joint trajectory commands
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.joint_names = {"caudal_joint", "joint1", "joint2"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {b1_, b2_, b3_};
        point.velocities = {v1, v2, v3};
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
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

        // Log fin angles, velocities, force, and pose
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Simulation time: %f", t);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fin angles: b1=%f, b2=%f, b3=%f", b1_, b2_, b3_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fin velocities: v1=%f, v2=%f, v3=%f", v1, v2, v3);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Estimated force: %f N", force_x);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Pose: x=%f, y=%f, z=%f", eta_(0), eta_(1), eta_(2));
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Published trajectory: b1=%f, b2=%f, b3=%f", b1_, b2_, b3_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Published caudal velocity: %f", v1);
    }

    // Member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;

    // Model parameters
    double m_, rho_, g_, a_, b_, L_;
    double xG_, yG_, zG_, xB_, yB_, zB_;
    double IxG_, IyG_, IzG_, Ixy_, Iyz_, Ixz_, Ix_, Iy_, Iz_;
    double x1_, y1_, z1_, x2_, y2_, z2_, x3_, y3_, z3_;
    double Cl_, Cd_, S1_, L_f1_, S2_, L_f2_, S3_, L_f3_;
    double PF1max_, freqmax_, dt_;
    double Ux_, Uy_, Uz_;
    double b1_, b2_, b3_;

    // Matrices and vectors
    Eigen::MatrixXd nu_, eta_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceDynamicNode>());
    rclcpp::shutdown();
    return 0;
}