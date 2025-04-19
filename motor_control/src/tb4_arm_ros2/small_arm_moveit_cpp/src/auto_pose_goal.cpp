#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "python_moveit_interface/srv/detect_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace srv = python_moveit_interface::srv;
using DetectPose = srv::DetectPose;

class AutoPoseService : public rclcpp::Node
{
public:
  AutoPoseService()
  : Node("auto_pose_service"),
    fixed_z_(0.015)  // 固定Z軸高度
  {
    // 初始化固定四元數（水平無旋轉）
    fixed_quaternion_.x = 0.0;
    fixed_quaternion_.y = 0.0;
    fixed_quaternion_.z = 0.0;
    fixed_quaternion_.w = 1.0;

    service_ = this->create_service<DetectPose>(
      "auto_pose_service",
      std::bind(&AutoPoseService::handle_service, this, _1, _2));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_pose_in_base", 10,
      std::bind(&AutoPoseService::pose_callback, this, _1));

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/auto_pose_status", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AutoPoseService::try_initialize_move_group, this));

    RCLCPP_INFO(this->get_logger(), "AutoPoseService initialized. XY buffer ready.");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Service<DetectPose>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // XY數據緩衝區
  std::deque<std::pair<double, double>> xy_buffer_;
  const double fixed_z_;
  geometry_msgs::msg::Quaternion fixed_quaternion_;

  void try_initialize_move_group()
  {
    if (move_group_) return;

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "small_arm");
      move_group_->setPlanningTime(10.0);
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
      timer_->cancel();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface... (%s)", e.what());
    }
  }

  void publish_status(const std::string &status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // 只提取XY座標
    double x = msg->position.x;
    double y = msg->position.y;
    
    xy_buffer_.push_back({x, y});
    
    // 保持緩衝區大小為10
    if (xy_buffer_.size() > 10) {
      xy_buffer_.pop_front();
    }

    RCLCPP_DEBUG(this->get_logger(), "XY updated: [%.4f, %.4f], buffer size: %zu",
                 x, y, xy_buffer_.size());
  }

  bool is_data_stable()
  {
    if (xy_buffer_.size() < 10) return false;

    // 計算XY波動範圍
    auto [min_x, max_x] = std::minmax_element(
      xy_buffer_.begin(), xy_buffer_.end(),
      [](const auto& a, const auto& b) { return a.first < b.first; });
    
    auto [min_y, max_y] = std::minmax_element(
      xy_buffer_.begin(), xy_buffer_.end(),
      [](const auto& a, const auto& b) { return a.second < b.second; });

    double x_range = max_x->first - min_x->first;
    double y_range = max_y->second - min_y->second;

    RCLCPP_DEBUG(this->get_logger(), "Stability check - X range: %.4f, Y range: %.4f",
                 x_range, y_range);

    return (x_range <= 0.01) && (y_range <= 0.01);
  }

  std::pair<double, double> calculate_average_xy()
  {
    double sum_x = 0.0;
    double sum_y = 0.0;
    
    for (const auto& point : xy_buffer_) {
      sum_x += point.first;
      sum_y += point.second;
    }
    
    return {sum_x / xy_buffer_.size(), sum_y / xy_buffer_.size()};
  }

  void handle_service(
    const std::shared_ptr<DetectPose::Request> /*request*/,
    std::shared_ptr<DetectPose::Response> response)
  {
    if (!move_group_) {
      response->success = false;
      response->message = "MoveGroupInterface not initialized.";
      publish_status(response->message);
      return;
    }

    // 檢查數據充足性
    if (xy_buffer_.size() < 10) {
      response->success = false;
      response->message = "Insufficient data (need 10 samples).";
      publish_status(response->message);
      return;
    }

    // 檢查數據穩定性
    if (!is_data_stable()) {
      response->success = false;
      response->message = "XY data not stable (range > 0.01m).";
      publish_status(response->message);
      return;
    }

    // 計算平均XY
    auto [avg_x, avg_y] = calculate_average_xy();
    
    // 構建目標姿勢
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = avg_x;
    target_pose.position.y = avg_y;
    target_pose.position.z = fixed_z_;
    target_pose.orientation = fixed_quaternion_;

    RCLCPP_INFO(this->get_logger(), 
                "Target pose:\nPosition: [%.4f, %.4f, %.4f]\nOrientation: [%.4f, %.4f, %.4f, %.4f]",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);

    // 運動規劃
    publish_status("Planning movement...");
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      publish_status("Executing movement...");
      move_group_->execute(plan);
      
      response->success = true;
      response->message = "Movement executed successfully";
      publish_status(response->message);
    } else {
      response->success = false;
      response->message = "Motion planning failed";
      publish_status(response->message);
    }

    // 清空緩衝區以準備下一輪
    xy_buffer_.clear();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoPoseService>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}