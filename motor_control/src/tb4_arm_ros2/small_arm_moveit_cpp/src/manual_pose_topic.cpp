#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PoseGoalNode : public rclcpp::Node
{
public:
  PoseGoalNode() : Node("pose_goal_node"), move_group_(shared_from_this(), "small_arm")
  {
    // 建立 Subscriber，接收 /target_pose topic
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_pose", 10,
      std::bind(&PoseGoalNode::poseCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PoseGoalNode 啟動完成，等待目標姿態...");
  }

private:
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "接收到目標 Pose，開始規劃...");

    move_group_.setPoseTarget(*msg);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "規劃成功，執行中...");
      move_group_.execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "規劃失敗！");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  moveit::planning_interface::MoveGroupInterface move_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseGoalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
