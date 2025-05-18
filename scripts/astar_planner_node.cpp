#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <Eigen/Dense>

struct AStarNode {
  int x, y; // Grid coordinates
  double g_cost; // Cost from start
  double h_cost; // Heuristic cost to goal
  AStarNode* parent;
  AStarNode(int x, int y, double g, double h, AStarNode* p = nullptr)
      : x(x), y(y), g_cost(g), h_cost(h), parent(p) {}
  double f_cost() const { return g_cost + h_cost; }
  bool operator>(const AStarNode& other) const { return f_cost() > other.f_cost(); }
};

class AStarPlannerNode : public rclcpp::Node {
public:
  AStarPlannerNode() : Node("astar_planner_node") {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Parameters
    grid_size_ = 0.1; // 10 cm resolution
    grid_width_ = 100; // 10m x 10m grid
    grid_height_ = 100;

    // Subscribers and Publishers
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/hpurv/pose", 10,
        std::bind(&AStarPlannerNode::PoseCallback, this, std::placeholders::_1));
    goal_pub_ = create_publisher<geometry_msgs::msg::Pose>(
        "/hpurv/goal_pose", 10);

    // Timer for planning (1 Hz)
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&AStarPlannerNode::PlanPath, this));

    // Initialize goal (example: 5m along x-axis)
    goal_pose_.position.x = 5.0;
    goal_pose_.position.y = 0.0;
    goal_pose_.position.z = 0.0;
    goal_pose_.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(), "AStarPlannerNode initialized");
  }

private:
  void PoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
  }

  void PlanPath() {
    // Convert current and goal positions to grid coordinates
    int start_x = std::round(current_pose_.position.x / grid_size_);
    int start_y = std::round(current_pose_.position.y / grid_size_);
    int goal_x = std::round(goal_pose_.position.x / grid_size_);
    int goal_y = std::round(goal_pose_.position.y / grid_size_);

    // A* algorithm
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::set<std::pair<int, int>> closed_list;
    open_list.emplace(start_x, start_y, 0.0, EuclideanDistance(start_x, start_y, goal_x, goal_y));

    std::vector<std::vector<AStarNode*>> nodes(grid_width_, std::vector<AStarNode*>(grid_height_, nullptr));

    while (!open_list.empty()) {
      AStarNode current = open_list.top();
      open_list.pop();
      int x = current.x, y = current.y;

      if (x == goal_x && y == goal_y) {
        // Reconstruct path
        std::vector<Eigen::Vector3d> path;
        AStarNode* node = nodes[x][y];
        while (node) {
          path.emplace_back(node->x * grid_size_, node->y * grid_size_, 0.0);
          node = node->parent;
        }
        std::reverse(path.begin(), path.end());

        // Publish first waypoint as goal
        if (!path.empty()) {
          geometry_msgs::msg::Pose goal;
          goal.position.x = path[0].x();
          goal.position.y = path[0].y();
          goal.position.z = path[0].z();
          goal.orientation.w = 1.0;
          goal_pub_->publish(goal);
          RCLCPP_INFO(get_logger(), "Published goal: x=%f, y=%f", goal.position.x, goal.position.y);
        }

        // Clean up
        for (auto& row : nodes)
          for (auto& node : row)
            delete node;
        return;
      }

      closed_list.emplace(x, y);
      if (!nodes[x][y]) nodes[x][y] = new AStarNode(current);

      // Check neighbors (4-connectivity)
      std::vector<std::pair<int, int>> neighbors = {
          {x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
      for (const auto& [nx, ny] : neighbors) {
        if (nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_ ||
            closed_list.count({nx, ny}))
          continue;

        double g_cost = current.g_cost + grid_size_;
        double h_cost = EuclideanDistance(nx, ny, goal_x, goal_y);
        if (!nodes[nx][ny] || g_cost < nodes[nx][ny]->g_cost) {
          if (nodes[nx][ny]) delete nodes[nx][ny];
          nodes[nx][ny] = new AStarNode(nx, ny, g_cost, h_cost, nodes[x][y]);
          open_list.emplace(nx, ny, g_cost, h_cost, nodes[x][y]);
        }
      }
    }

    RCLCPP_WARN(get_logger(), "No path found to goal");
    for (auto& row : nodes)
      for (auto& node : row)
        delete node;
  }

  double EuclideanDistance(int x1, int y1, int x2, int y2) {
    return grid_size_ * std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose current_pose_, goal_pose_;
  double grid_size_;
  int grid_width_, grid_height_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}