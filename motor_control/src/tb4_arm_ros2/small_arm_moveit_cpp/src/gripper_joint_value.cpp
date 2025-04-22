#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include "python_moveit_interface/srv/gripper_control.hpp"

using GripperControl = python_moveit_interface::srv::GripperControl;

class GripperControlService : public rclcpp::Node
{
public:
  GripperControlService()
  : Node("gripper_control_service"),
    open_position_(-0.55),
    close_position_(0.0),
    position_tolerance_(0.05)
  {
    gripper_service_ = this->create_service<GripperControl>(
      "gripper_control_service",
      [this](const std::shared_ptr<GripperControl::Request> req,
             std::shared_ptr<GripperControl::Response> res) {
        this->handle_service(req, res);
      });

    error_publisher_ = this->create_publisher<std_msgs::msg::String>("/gripper_error", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { this->try_initialize_move_group(); });

    RCLCPP_INFO(this->get_logger(), "\xf0\x9f\x9b\xa0 Gripper control node started");
  }

private:
  const std::string joint_name_ = "gripper_joint1";
  const double open_position_;
  const double close_position_;
  const double position_tolerance_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<GripperControl>::SharedPtr gripper_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_publisher_;

  void try_initialize_move_group()
  {
    if (move_group_interface_) return;

    try {
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

      move_group_interface_->setMaxVelocityScalingFactor(0.8);
      move_group_interface_->setMaxAccelerationScalingFactor(0.8);
      move_group_interface_->setPlanningTime(2.0);
      move_group_interface_->setGoalJointTolerance(position_tolerance_);

      if (!move_group_interface_->startStateMonitor(5.0)) {
        RCLCPP_WARN(this->get_logger(), "\xe2\x9a\xa0 Failed to start state monitor");
      }

      RCLCPP_INFO(this->get_logger(), "\xe2\x9c\x85 MoveGroup interface initialized");

      const auto joint_names = move_group_interface_->getJointNames();
      RCLCPP_INFO(this->get_logger(), "\xf0\x9f\x94\x8d Dumping joints in MoveGroupInterface:");
      for (size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, joint_names[i].c_str());
      }

      timer_->cancel();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
    }
  }

  void publish_error(const std::string& msg)
  {
    auto error_msg = std_msgs::msg::String();
    error_msg.data = "[GRIPPER ERROR] " + msg;
    error_publisher_->publish(error_msg);
  }

  void handle_service(
    const std::shared_ptr<GripperControl::Request> request,
    std::shared_ptr<GripperControl::Response> response)
  {
    if (!move_group_interface_) {
      const std::string error_msg = "MoveGroup interface not ready";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      response->success = false;
      response->message = error_msg;
      publish_error(error_msg);
      return;
    }

    const double target = request->close ? close_position_ : open_position_;
    RCLCPP_INFO(this->get_logger(), "\xf0\x9f\x93\xa5 Received command: %s (Target: %.3f)",
                request->close ? "CLOSE" : "OPEN", target);

    try {
      move_group_interface_->setJointValueTarget(joint_name_, target);
      auto result = move_group_interface_->move();

      std::vector<double> current_joint_values;
      const int max_retries = 30;
      for (int i = 0; i < max_retries; ++i) {
        current_joint_values = move_group_interface_->getCurrentJointValues();
        if (!current_joint_values.empty()) break;
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }

      if (current_joint_values.empty()) {
        throw std::runtime_error("Current joint values still empty after retries");
      }

      const auto joint_names = move_group_interface_->getJointNames();
      auto it = std::find(joint_names.begin(), joint_names.end(), joint_name_);
      if (it == joint_names.end()) {
        throw std::runtime_error("Joint '" + joint_name_ + "' not found in joint list");
      }

      size_t index = std::distance(joint_names.begin(), it);
      if (index >= current_joint_values.size()) {
        throw std::runtime_error("Index out of range for joint position vector");
      }

      double actual = current_joint_values.at(index);
      double error = std::abs(actual - target);

      RCLCPP_INFO(this->get_logger(), "\xf0\x9f\x9f\xa2 Actual: %.4f | Target: %.4f | Error: %.4f", actual, target, error);

      response->success = (error < position_tolerance_);
      response->message = response->success ? "Operation succeeded" : "Error exceeds tolerance";

    } catch (const std::exception& e) {
      const std::string error_msg = "Control failed: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      response->success = false;
      response->message = error_msg;
      publish_error(error_msg);
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
