#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "python_moveit_interface/srv/pose_request.hpp"

using PoseRequest = python_moveit_interface::srv::PoseRequest;

class NamedGoalService : public rclcpp::Node {
public:
  NamedGoalService()
  : Node("named_goal_service")
  {
    // 建立 ROS2 服務，用於接收目標名稱並執行對應的機械手臂動作
    service_ = this->create_service<PoseRequest>(
      "named_pose_service",
      std::bind(&NamedGoalService::handle_goal_request, this, std::placeholders::_1, std::placeholders::_2));

    // 每 100 毫秒檢查一次 MoveGroupInterface 是否可以初始化
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NamedGoalService::try_initialize_move_group, this));

    RCLCPP_INFO(this->get_logger(), "Named goal node created, waiting for MoveGroupInterface...");
  }

private:
  // 嘗試初始化 MoveGroupInterface（控制 MoveIt 的關鍵元件）
  void try_initialize_move_group()
  {
    if (move_group_interface_) return;

    try {
      // 嘗試初始化 MoveGroupInterface，對應機械手臂的 move_group 名稱為 "small_arm"
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "small_arm");

      move_group_interface_->setPlanningTime(5.0);
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
      timer_->cancel();  // 初始化成功後停止定時器
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface... (%s)", e.what());
    }
  }

  // 當服務被呼叫時執行，處理前往命名目標的請求
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

    // 取得使用者請求的目標名稱
    std::string target_name = request->message;
    RCLCPP_INFO(this->get_logger(), "Received named target: %s", target_name.c_str());

    // 設定目標為命名的姿態（必須事先在 MoveIt SRDF 中定義）
    move_group_interface_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // 嘗試規劃路徑
    if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
      move_group_interface_->execute(plan);
      response->success = true;
      response->message = "Executed: " + target_name;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      response->success = false;
      response->message = "Planning failed: " + target_name;
    }
  }

  // ROS2 service
  rclcpp::Service<PoseRequest>::SharedPtr service_;

  // Moveit interface, moveing arm 
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  //initialize timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NamedGoalService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}