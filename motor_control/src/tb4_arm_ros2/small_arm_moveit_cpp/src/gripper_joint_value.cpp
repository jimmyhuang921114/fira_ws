#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "python_moveit_interface/srv/gripper_control.hpp"

using GripperControl = python_moveit_interface::srv::GripperControl;

class GripperControlService : public rclcpp::Node
{
public:
  GripperControlService()
  : Node("gripper_control_service")
  {
    gripper_service_ = this->create_service<GripperControl>(
      "gripper_control_service",
      std::bind(&GripperControlService::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GripperControlService::try_initialize_move_group, this));

    RCLCPP_INFO(this->get_logger(), "GripperControlService created, waiting for MoveGroupInterface...");
  }

private:
  void try_initialize_move_group()
  {
    if (move_group_interface_) return;

    try {
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
      timer_->cancel();
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface... (%s)", e.what());
    }
  }

  void handle_service(
    const std::shared_ptr<GripperControl::Request> request,
    std::shared_ptr<GripperControl::Response> response)
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized.");
      response->success = false;
      response->message = "MoveGroupInterface not ready.";
      return;
    }

    double target_position = request->close ? -0.5235 : 0.0698;  // Updated closed/open positions
    move_group_interface_->setJointValueTarget("robotiq_85_left_knuckle_joint", target_position);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing gripper movement...");

      auto result = move_group_interface_->execute(plan);
      move_group_interface_->stop();
      rclcpp::sleep_for(std::chrono::milliseconds(300));

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Gripper moved to %.3f rad successfully.", target_position);
        response->success = true;
        response->message = "Gripper moved successfully.";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed (code = %d).", result.val);
        response->success = false;
        response->message = "Gripper execution failed.";
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      response->success = false;
      response->message = "Gripper planning failed.";
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<GripperControl>::SharedPtr gripper_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlService>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
