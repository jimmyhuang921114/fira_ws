#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "python_moveit_interface/srv/pose_request.hpp"  // 替換成實際 srv 的路徑

using PoseRequest = python_moveit_interface::srv::PoseRequest;

class NamedGoalService : public rclcpp::Node {
public:
  NamedGoalService() : Node("named_goal_service") {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "small_arm");

    service_ = this->create_service<PoseRequest>(
      "named_goal_service",
      std::bind(&NamedGoalService::handle_goal_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "NamedGoalService ready: Waiting for service calls");
  }

private:
  void handle_goal_request(
    const std::shared_ptr<PoseRequest::Request> request,
    std::shared_ptr<PoseRequest::Response> response)
  {
    std::string target_name = request->message;
    RCLCPP_INFO(this->get_logger(), "Received service target: %s", target_name.c_str());

    move_group_interface_->setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
      move_group_interface_->execute(plan);
      response->success = true;
      response->message = "Executed: " + target_name;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
      response->success = false;
      response->message = "Planning failed: " + target_name;
    }
  }

  rclcpp::Service<PoseRequest>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NamedGoalService>());
  rclcpp::shutdown();
  return 0;
}
