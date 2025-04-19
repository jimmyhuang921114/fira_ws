#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // 初始化 ROS 2 節點
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("gripper_joint_value");

  // 設定 logger（用於輸出訊息）
  auto const logger = rclcpp::get_logger("gripper_joint_value");

  // 建立 MoveGroupInterface，目標群組為 "gripper"
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gripper");

  // 設定目標關節角度：robotiq_85_left_knuckle_joint 設定為 0.803 弧度
  // 根據實際模型，這可能代表夾爪完全閉合或張開
  move_group_interface.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.803);

  // 建立計劃（plan）物件，並嘗試規劃軌跡
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // 若規劃成功則執行此動作
  if(success) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // 結束 ROS2
  rclcpp::shutdown();
  return 0;
}
