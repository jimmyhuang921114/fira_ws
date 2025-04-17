#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "python_moveit_interface/srv/pose_request.hpp"

using PoseRequest = python_moveit_interface::srv::PoseRequest;

class NamedGoalService : public rclcpp::Node {
public:
  NamedGoalService()
  : Node("named_goal_service")
  {
    service_ = this->create_service<PoseRequest>(
      "named_pose_service",  // ✅ 與 router 對應的 service 名稱
      std::bind(&NamedGoalService::handle_goal_request, this,
                std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NamedGoalService::try_initialize_move_group, this));

    RCLCPP_INFO(this->get_logger(), "🛠️ Node initialized, waiting for MoveGroupInterface...");
  }

private:
  void try_initialize_move_group() {
    if (move_group_interface_) return;

    try {
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "small_arm");
      move_group_interface_->setPlanningTime(5.0);
      RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized.");
      timer_->cancel();
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "⏳ Waiting for full node init...");
    }
  }

  void handle_goal_request(
    const std::shared_ptr<PoseRequest::Request> request,
    std::shared_ptr<PoseRequest::Response> response)
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized.");
      response->success = false;
      response->message = "MoveGroupInterface not ready.";
      return;
    }

    std::string target_name = request->message;
    RCLCPP_INFO(this->get_logger(), "📥 Received named target: %s", target_name.c_str());

    move_group_interface_->setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "✅ Planning succeeded, executing...");
      move_group_interface_->execute(plan);
      response->success = true;
      response->message = "Executed: " + target_name;
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Planning failed");
      response->success = false;
      response->message = "Planning failed: " + target_name;
    }
  }

  rclcpp::Service<PoseRequest>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NamedGoalService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
