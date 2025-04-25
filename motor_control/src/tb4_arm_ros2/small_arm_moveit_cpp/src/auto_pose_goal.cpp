#include <memory>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "python_moveit_interface/srv/detect_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace srv = python_moveit_interface::srv;
using DetectPose = srv::DetectPose;

class AutoPoseService : public rclcpp::Node {
public:
  AutoPoseService()
  : Node("auto_pose_service"), has_pose_(false)
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

    RCLCPP_INFO(this->get_logger(), "AutoPoseService initialized.");
  }

private:
  std::mutex move_group_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Service<DetectPose>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::deque<geometry_msgs::msg::Pose> pose_buffer_;
  static constexpr size_t BUFFER_SIZE = 30;
  bool has_pose_ = false;

  void try_initialize_move_group()
  {
    if (move_group_) return;

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "small_arm");
      move_group_->setPlanningTime(5.0);
      move_group_->setNumPlanningAttempts(3);
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
    if (pose_buffer_.size() >= BUFFER_SIZE)
      pose_buffer_.pop_front();
    pose_buffer_.push_back(*msg);
    has_pose_ = true;

    // RCLCPP_DEBUG(this->get_logger(),
    //   "Pose received: x=%.3f y=%.3f z=%.3f", msg->position.x, msg->position.y, msg->position.z);
  }

  geometry_msgs::msg::Pose average_pose()
  {
    geometry_msgs::msg::Pose avg;
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto &p : pose_buffer_) {
      sum_x += p.position.x;
      sum_y += p.position.y;
      sum_z += p.position.z;
    }
    avg.position.x = sum_x / pose_buffer_.size();
    avg.position.y = sum_y / pose_buffer_.size();
    avg.position.z = sum_z / pose_buffer_.size();
    // avg.position.x = 0.000;
    // avg.position.y = 0.200;
    // avg.position.z = 0.07;
    const auto &p = pose_buffer_.back();
    if (avg.position.x > 0){
      avg.orientation.x = 0;
      avg.orientation.y = 0;
      avg.orientation.z = -0.707;
      avg.orientation.w = 0.707;}
    else{
      avg.orientation.x = 0;
      avg.orientation.y = 0;
      avg.orientation.z = 0.707;
      avg.orientation.w = 0.707;}
      
    RCLCPP_DEBUG(this->get_logger(),
    "Averaged Position: x=%.3f y=%.3f z=%.3f",
    avg.position.x, avg.position.y, avg.position.z);

    RCLCPP_DEBUG(this->get_logger(),"Orientation: x=%.3f y=%.3f z=%.3f w=%.3f", avg.orientation.x, avg.orientation.y, avg.orientation.z, avg.orientation.w);
      return avg;
  }

  void handle_service(
    const std::shared_ptr<DetectPose::Request>,
    std::shared_ptr<DetectPose::Response> response)
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (!move_group_) {
      response->success = false;
      response->message = "MoveGroupInterface not initialized.";
      publish_status(response->message);
      return;
    }

    if (!has_pose_ || pose_buffer_.size() < BUFFER_SIZE) {
      response->success = false;
      response->message = "Insufficient pose data.";
      publish_status(response->message);
      return;
    }

    auto target_pose = average_pose();

    if (target_pose.position.z < 0.01 || target_pose.position.z > 0.3) {
      response->success = false;
      response->message = "Target Z out of range.";
      publish_status(response->message);  
      return;
    }

    publish_status("Planning...");

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      publish_status("Executing...");
      auto result = move_group_->execute(plan);
      move_group_->stop();
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = true;
        response->message = "Motion complete.";

        RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for pose to stabilize...");
        pose_buffer_.clear();
        rclcpp::sleep_for(std::chrono::seconds(5));  
      } else {
        response->success = false;
        response->message = "Execution failed.";
      }
    }
    
    publish_status(response->message);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoPoseService>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
