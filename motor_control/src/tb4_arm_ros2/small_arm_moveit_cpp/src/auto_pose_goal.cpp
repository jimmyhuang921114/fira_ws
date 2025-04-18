#include <memory>
#include <vector>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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
  : Node("auto_pose_service")
  {
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

    RCLCPP_INFO(this->get_logger(), "🛠️ AutoPoseService created, waiting for MoveGroupInterface...");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Service<DetectPose>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<geometry_msgs::msg::Pose> pose_buffer_;

  void try_initialize_move_group()
  {
    if (move_group_) return;

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "small_arm");
      move_group_->setPlanningTime(10.0);
      RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized.");
      timer_->cancel();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "⏳ Waiting for MoveGroupInterface... (%s)", e.what());
    }
  }

  void publish_status(const std::string &status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "📣 Status: %s", status.c_str());
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    pose_buffer_.push_back(*msg);
    if (pose_buffer_.size() > 10)
      pose_buffer_.erase(pose_buffer_.begin());

    RCLCPP_INFO(this->get_logger(), "📥 Received Pose: x=%.4f y=%.4f z=%.4f",
                msg->position.x, msg->position.y, msg->position.z);
  }

  bool is_pose_stable(double tolerance = 0.009)
  {
    if (pose_buffer_.size() < 10) return false;

    auto range = [&](auto accessor) {
      std::vector<double> values;
      for (const auto &p : pose_buffer_) values.push_back(accessor(p));
      auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
      return *max_it - *min_it;
    };

    return range([](auto p){ return p.position.x; }) <= tolerance &&
           range([](auto p){ return p.position.y; }) <= tolerance &&
           range([](auto p){ return p.position.z; }) <= tolerance;
  }

  geometry_msgs::msg::Pose average_pose()
  {
    geometry_msgs::msg::Pose avg;
    for (const auto &p : pose_buffer_) {
      avg.position.x += p.position.x;
      avg.position.y += p.position.y;
      avg.position.z += p.position.z;
    }

    avg.position.x /= pose_buffer_.size();
    avg.position.y /= pose_buffer_.size();

    // ✅ 固定 Z 值以確保抓取穩定
    avg.position.z = 0.015;

    avg.orientation = pose_buffer_.back().orientation;

    RCLCPP_INFO(this->get_logger(),
      "📊 Averaged Pose: x=%.4f y=%.4f z=%.4f | orientation=[%.4f, %.4f, %.4f, %.4f]",
      avg.position.x, avg.position.y, avg.position.z,
      avg.orientation.x, avg.orientation.y, avg.orientation.z, avg.orientation.w);

    RCLCPP_WARN(this->get_logger(), "⚠️ Z value is set to fixed 0.015 m");
    return avg;
  }

  void handle_service(
    const std::shared_ptr<DetectPose::Request> /*request*/,
    std::shared_ptr<DetectPose::Response> response)
  {
    if (!move_group_) {
      response->success = false;
      response->message = "❌ MoveGroupInterface not initialized.";
      publish_status(response->message);
      return;
    }

    publish_status("⏳ Buffering pose data...");
    rclcpp::Rate rate(20);
    auto start_time = this->now();
    const double timeout = 10.0;

    while (rclcpp::ok() && pose_buffer_.size() < 10) {
      if ((this->now() - start_time).seconds() > timeout) {
        response->success = false;
        response->message = "❌ Timeout: Not enough pose data";
        publish_status(response->message);
        return;
      }
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }

    publish_status("🔍 Checking pose stability...");
    start_time = this->now();
    while (rclcpp::ok() && !is_pose_stable()) {
      if ((this->now() - start_time).seconds() > timeout) {
        response->success = false;
        response->message = "❌ Timeout: Pose not stable";
        publish_status(response->message);
        return;
      }
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }

    geometry_msgs::msg::Pose pose = average_pose();

    publish_status("🧠 Planning motion...");
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pose);

    RCLCPP_INFO(this->get_logger(),
      "🎯 Target Pose: x=%.3f y=%.3f z=%.3f | orientation=[%.3f, %.3f, %.3f, %.3f]",
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      publish_status("✅ Motion plan successful, executing...");
      move_group_->execute(plan);
      response->success = true;
      response->message = "✅ Motion executed successfully";
      publish_status(response->message);
    } else {
      response->success = false;
      response->message = "❌ Motion planning failed";
      publish_status(response->message);
    }
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
