#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "python_moveit_interface/srv/pose_request.hpp"  // 替換成你實際 package
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using PoseRequest = python_moveit_interface::srv::PoseRequest;


class AutoPoseService : public rclcpp::Node
{
public:
  AutoPoseService()
  : Node("auto_pose_service"),
    move_group_(shared_from_this(), "small_arm")
  {
    service_ = this->create_service<python_moveit_interface::srv::PoseRequest>(
      "auto_pose_service", std::bind(&AutoPoseService::handle_service, this, _1, _2));

    move_group_.setPlanningTime(5.0);
    RCLCPP_INFO(this->get_logger(), "✅ auto_pose_service is ready.");
  }

private:
  void handle_service(
    const std::shared_ptr<PoseRequest::Request> request,
    std::shared_ptr<PoseRequest::Response> response)
  {
    auto pose = request->target_pose;

    RCLCPP_INFO(this->get_logger(), "📥 Received pose x=%.2f y=%.2f z=%.2f | orientation=%.2f,%.2f,%.2f,%.2f",
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    if (pose.orientation.x == 0.0 && pose.orientation.y == 0.0 &&
        pose.orientation.z == 0.0 && pose.orientation.w == 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "❌ Invalid orientation received (0,0,0,0)");
      response->success = false;
      response->message = "Invalid quaternion orientation (0,0,0,0).";
      return;
    }

    // 可擴充：如有需要在此加入 base ↔ camera TF 或補預設姿態的轉換

    move_group_.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan);
      response->success = true;
      response->message = "Pose execution success.";
    } else {
      response->success = false;
      response->message = "Pose planning failed.";
    }
  }

  rclcpp::Service<python_moveit_interface::srv::PoseRequest>::SharedPtr service_;
  moveit::planning_interface::MoveGroupInterface move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoPoseService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
