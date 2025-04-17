#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "python_moveit_interface/srv/gripper_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class GripperControlService : public rclcpp::Node
{
public:
  GripperControlService()
  : Node("gripper_control_service")
  {
    gripper_service_ = this->create_service<python_moveit_interface::srv::GripperControl>(
      "gripper_control_service", std::bind(&GripperControlService::handle_service, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "🛠️ Node initialized, waiting for MoveGroupInterface...");
  }

  void initialize()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized.");
  }

private:
  void handle_service(
    const std::shared_ptr<python_moveit_interface::srv::GripperControl::Request> request,
    std::shared_ptr<python_moveit_interface::srv::GripperControl::Response> response)
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized!");
      response->success = false;
      response->message = "MoveGroupInterface not initialized.";
      return;
    }

    double target_value = request->close ? 0.0873 : -0.5236;
    std::string action = request->close ? "close" : "open";

    RCLCPP_INFO(this->get_logger(), "Received gripper command: %s (target=%.2f)", action.c_str(), target_value);

    move_group_interface_->setJointValueTarget("robotiq_85_left_knuckle_joint", target_value);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Gripper executed %s successfully.", action.c_str());
      response->success = true;
      response->message = "Gripper executed successfully.";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Gripper planning failed.");
      response->success = false;
      response->message = "Gripper planning failed.";
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<python_moveit_interface::srv::GripperControl>::SharedPtr gripper_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlService>();
  node->initialize();  // ✅ 這裡才安全地使用 shared_from_this()
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
