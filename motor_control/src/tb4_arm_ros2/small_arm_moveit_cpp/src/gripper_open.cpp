#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class GripperControlNode : public rclcpp::Node
{
public:
  GripperControlNode()
  : Node("gripper_control_node")
  {
    gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/gripper_command", 10,
      std::bind(&GripperControlNode::gripperCallback, this, std::placeholders::_1));
  }

  void init()
  {
    // 修正版本：使用 this->shared_from_this()
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "gripper");
  }


private:
  void gripperCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized yet!");
      return;
    }

    double target_value = msg->data ? 0.0 : 3.0;
    std::string action = msg->data ? "close" : "open";

    RCLCPP_INFO(this->get_logger(), "Received command: %s (target=%.2f)", action.c_str(), target_value);

    // 設定目標關節角度
    move_group_interface_->setJointValueTarget("robotiq_85_left_knuckle_joint", target_value);

    // 規劃並執行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Gripper executed %s successfully.", action.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Gripper planning failed.");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_cmd_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperControlNode>();
  node->init();  // 初始化 MoveGroupInterface

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
