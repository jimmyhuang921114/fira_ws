#include <memory>
#include <future>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "python_moveit_interface/srv/gripper_control.hpp"

using GripperControl = python_moveit_interface::srv::GripperControl;

class GripperControlServiceWithDiagnostics : public rclcpp::Node
{
public:
  GripperControlServiceWithDiagnostics()
  : Node("gripper_control_service_with_diagnostics")
  {
    gripper_service_ = this->create_service<GripperControl>(
      "gripper_control_service",
      std::bind(&GripperControlServiceWithDiagnostics::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    diagnostic_publisher_ = this->create_publisher<std_msgs::msg::String>("/gripper_diagnostic", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GripperControlServiceWithDiagnostics::try_initialize_move_group, this));

    RCLCPP_INFO(this->get_logger(), "Gripper diagnostic node started, waiting for MoveGroupInterface...");
  }

private:
  std::mutex move_group_mutex_;
  void try_initialize_move_group()
  {
    if (move_group_interface_) return;

    try {
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for gripper initialized.");
      timer_->cancel();
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface... (%s)", e.what());
    }
  }

  void publish_diagnostic(const std::string &message)
  {
    std_msgs::msg::String msg;
    msg.data = message;
    diagnostic_publisher_->publish(msg);
  }

  void reset_move_group_state() {
    move_group_interface_->stop();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->setStartStateToCurrentState();
    RCLCPP_INFO(this->get_logger(), "MoveGroup state reset.");
  }

  void handle_service(
    const std::shared_ptr<GripperControl::Request> request,
    std::shared_ptr<GripperControl::Response> response)
  {
  std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (!move_group_interface_) {
      std::string msg = "MoveGroupInterface not initialized.";
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      publish_diagnostic(msg);
      response->success = false;
      response->message = msg;
      return;
    }

    const std::string joint_name = "gripper_joint1";
    const double target_val = request->close ? 0.0174 : -0.510;
    const std::string action = request->close ? "close" : "open";

    std::string msg = "Gripper command received: " + action + " → " + std::to_string(target_val);
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
    publish_diagnostic(msg);

    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setGoalTolerance(0.05);
    move_group_interface_->setJointValueTarget(joint_name, target_val);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto planning_result = move_group_interface_->plan(plan);

    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
      publish_diagnostic("Planning succeeded. Executing...");
      auto execution_result = move_group_interface_->execute(plan);

      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        msg = "Gripper " + action + " succeeded.";
        reset_move_group_state(); 
        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        publish_diagnostic(msg);
        response->success = true;
        response->message = msg;
        return;
      } else {
        msg = "Execution failed (code: " + std::to_string(execution_result.val) + ")";
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        publish_diagnostic(msg);
        reset_move_group_state();
        rclcpp::sleep_for(std::chrono::seconds(2));
      }
    } else {
      msg = "Planning failed.";
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      publish_diagnostic(msg);
      reset_move_group_state();
    }

    const auto joint_names = move_group_interface_->getJointNames();
    const auto joint_values = move_group_interface_->getCurrentJointValues();

    if (joint_names.empty() || joint_values.empty()) {
      msg = "Joint state data is empty. Possibly controller failure.";
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      publish_diagnostic(msg);
      response->success = false;
      response->message = msg;
      return;
    }

    auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (it != joint_names.end()) {
      size_t idx = std::distance(joint_names.begin(), it);
      double actual = joint_values[idx];
      double error = std::abs(actual - target_val);

      msg = "Joint " + joint_name + " error = " + std::to_string(error) +
            " (actual = " + std::to_string(actual) +
            ", target = " + std::to_string(target_val) + ")";
      RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
      publish_diagnostic(msg);

      if (error < 0.1) {
        msg = "Execution failed but within acceptable range. Treating as success.";
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        publish_diagnostic(msg);
        response->success = true;
        response->message = msg;
        return;
      }
    } else {
      msg = "Joint '" + joint_name + "' not found in joint state.";
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      publish_diagnostic(msg);
    }

    response->success = false;
    response->message = "Execution failed or too far from target.";
    publish_diagnostic(response->message);
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<GripperControl>::SharedPtr gripper_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlServiceWithDiagnostics>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
