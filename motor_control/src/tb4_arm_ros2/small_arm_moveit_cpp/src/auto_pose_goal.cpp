#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "python_moveit_interface/srv/pose_request.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <numeric>

using std::placeholders::_1;
using std::placeholders::_2;
using PoseRequest = python_moveit_interface::srv::PoseRequest;

class AutoPoseService : public rclcpp::Node
{
public:
  AutoPoseService()
  : Node("auto_pose_service")
  {
    service_ = this->create_service<PoseRequest>(
      "auto_pose_service", std::bind(&AutoPoseService::handle_service, this, _1, _2));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/text_coordinate", 10, std::bind(&AutoPoseService::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "[INFO] Node initialized, waiting for MoveGroupInterface...");
  }

  void initialize()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "small_arm"); 

    move_group_->setPlanningTime(5.0);
    RCLCPP_INFO(this->get_logger(), "[OK] MoveGroupInterface initialized.");
  }

private:
  std::vector<geometry_msgs::msg::Pose> pose_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<PoseRequest>::SharedPtr service_;

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    pose_buffer_.push_back(*msg);
    if (pose_buffer_.size() > 10)
      pose_buffer_.erase(pose_buffer_.begin());
  }

  bool is_pose_stable(double tolerance)
  {
    if (pose_buffer_.size() < 10)
      return false;

    auto extract = [](auto f) {
      return [=](const geometry_msgs::msg::Pose &p) { return f(p); };
    };

    auto range = [&](auto accessor) {
      std::vector<double> values;
      for (const auto &p : pose_buffer_) values.push_back(accessor(p));
      auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
      return *max_it - *min_it;
    };

    return range(extract([](auto p) { return p.position.x; })) <= tolerance &&
           range(extract([](auto p) { return p.position.y; })) <= tolerance &&
           range(extract([](auto p) { return p.position.z; })) <= tolerance;
  }

  geometry_msgs::msg::Pose average_pose()
  {
    geometry_msgs::msg::Pose avg;
    for (const auto &p : pose_buffer_)
    {
      avg.position.x += p.position.x;
      avg.position.y += p.position.y;
      avg.position.z += p.position.z;
    }
    avg.position.x /= pose_buffer_.size();
    avg.position.y /= pose_buffer_.size();
    avg.position.z /= pose_buffer_.size();
    avg.orientation = pose_buffer_.back().orientation;
    return avg;
  }

  void handle_service(
    const std::shared_ptr<PoseRequest::Request> request,
    std::shared_ptr<PoseRequest::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "[INFO] Received request, checking for stable pose...");
    rclcpp::Rate rate(20);
    int attempts = 0;
    const int max_attempts = 200;

    while (rclcpp::ok() && !is_pose_stable(0.005) && attempts++ < max_attempts)
    {
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }

    if (!is_pose_stable(0.005))
    {
      RCLCPP_ERROR(this->get_logger(), "[FAIL] Stable pose not found within time limit.");
      response->success = false;
      response->message = "Stable pose not found.";
      return;
    }

    geometry_msgs::msg::Pose pose = average_pose();
    RCLCPP_INFO(this->get_logger(), "[OK] Stable pose acquired: x=%.3f y=%.3f z=%.3f", pose.position.x, pose.position.y, pose.position.z);

    move_group_->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group_->execute(plan);
      response->success = true;
      response->message = "Pose execution success.";
    }
    else
    {
      response->success = false;
      response->message = "Pose planning failed.";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoPoseService>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}