#include <memory>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "python_moveit_interface/srv/detect_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using DetectPose = python_moveit_interface::srv::DetectPose;

class CartesianPathServer : public rclcpp::Node
{
public:
  CartesianPathServer()
  : Node("cartesian_path_server"), fixed_z_(0.03)
  {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "small_arm");

    service_ = this->create_service<DetectPose>(
      "compute_cartesian_path",
      std::bind(&CartesianPathServer::handle_service, this, _1, _2));

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_pose_in_base", 10,
      std::bind(&CartesianPathServer::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Cartesian Path Service with dynamic Z adjustment is ready.");
  }

private:
  geometry_msgs::msg::Point latest_point_;
  std::mutex point_mutex_;
  double fixed_z_;

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(point_mutex_);
    latest_point_ = msg->position;

    double z = msg->position.z;

    RCLCPP_INFO(this->get_logger(), "Received pose (%.3f, %.3f, %.3f) → fixed_z = %.3f",
                latest_point_.x, latest_point_.y, latest_point_.z, fixed_z_);
  }

  void handle_service(
    const std::shared_ptr<DetectPose::Request> request,
    std::shared_ptr<DetectPose::Response> response)
  {
    std::lock_guard<std::mutex> lock(point_mutex_);

    auto start_pose = move_group_interface_->getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.x = latest_point_.x;
    target_pose.position.y = latest_point_.y;
    target_pose.position.z = fixed_z_;

    // 固定 Z 軸朝下姿態
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);  // Z 軸朝下
    target_pose.orientation = tf2::toMsg(q);

    std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;

    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Cartesian Path Planning (%.2f%%)", fraction * 100.0);

    if (fraction > 0.99) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      if (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = true;
        response->message = "Executed successfully.";
        return;
      }
    }

    response->success = false;
    response->message = "Cartesian path planning or execution failed.";
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<DetectPose>::SharedPtr service_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
