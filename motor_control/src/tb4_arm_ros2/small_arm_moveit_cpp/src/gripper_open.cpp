#include <memory>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "python_moveit_interface/srv/gripper_control.hpp"

using GripperControl = python_moveit_interface::srv::GripperControl;

class GripperControlService : public rclcpp::Node
{
public:
  GripperControlService()
  : Node("gripper_control_service")
  {
    gripper_service_ = this->create_service<GripperControl>(
      "gripper_control_service",
      std::bind(&GripperControlService::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GripperControlService::try_initialize_move_group, this));

    RCLCPP_INFO(this->get_logger(), "🛠️ Gripper node created, waiting for MoveGroupInterface...");
  }

private:
  void try_initialize_move_group()
  {
    if (move_group_interface_) return;

    try {
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");

      auto targets = move_group_interface_->getNamedTargets();
      for (const auto& t : targets)
        RCLCPP_INFO(this->get_logger(), "Named target available: %s", t.c_str());

      RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized.");
      timer_->cancel();
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "⏳ Waiting for MoveGroupInterface... (%s)", e.what());
    }
  }

  void handle_service(
    const std::shared_ptr<GripperControl::Request> request,
    std::shared_ptr<GripperControl::Response> response)
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized.");
      response->success = false;
      response->message = "MoveGroupInterface not ready.";
      return;
    }

    std::string target_name = request->close ? "close" : "open";
    RCLCPP_INFO(this->get_logger(), "📥 Received gripper command: %s", target_name.c_str());

    auto named_targets = move_group_interface_->getNamedTargets();
    if (std::find(named_targets.begin(), named_targets.end(), target_name) == named_targets.end()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Named target '%s' not found!", target_name.c_str());
      response->success = false;
      response->message = "Named target not found.";
      return;
    }

    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->setPlanningTime(10.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);
    move_group_interface_->setGoalTolerance(0.01);

    move_group_interface_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "✅ Planning succeeded. Executing gripper asynchronously...");

      auto future_result = std::async(std::launch::async, [this, plan](){
        return move_group_interface_->execute(plan);
      });

      auto status = future_result.wait_for(std::chrono::seconds(5));

      if (status == std::future_status::ready &&
          future_result.get() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ Gripper executed '%s' successfully.", target_name.c_str());
        response->success = true;
        response->message = "Gripper executed successfully.";
      } else {
        move_group_interface_->stop();
        RCLCPP_ERROR(this->get_logger(), "❌ Gripper execution failed or timed out.");
        response->success = false;
        response->message = "Gripper execution failed or timed out.";
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Gripper planning failed.");
      response->success = false;
      response->message = "Gripper planning failed.";
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Service<GripperControl>::SharedPtr gripper_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlService>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}


// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include "python_moveit_interface/srv/gripper_control.hpp"

// using GripperControl = python_moveit_interface::srv::GripperControl;

// class GripperControlService : public rclcpp::Node
// {
// public:
//   GripperControlService()
//   : Node("gripper_control_service")
//   {
//     gripper_service_ = this->create_service<GripperControl>(
//       "gripper_control_service",
//       std::bind(&GripperControlService::handle_service, this, std::placeholders::_1, std::placeholders::_2));

//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(100),
//       std::bind(&GripperControlService::try_initialize_move_group, this));

//     RCLCPP_INFO(this->get_logger(), "GripperControlService created, waiting for MoveGroupInterface...");
//   }

// private:
//   void try_initialize_move_group()
//   {
//     if (move_group_interface_) return;

//     try {
//       move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//         shared_from_this(), "gripper");

//       RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
//       timer_->cancel();
//     } catch (const std::exception& e) {
//       RCLCPP_WARN(this->get_logger(), "Waiting for MoveGroupInterface... (%s)", e.what());
//     }
//   }

//   void handle_service(
//     const std::shared_ptr<GripperControl::Request> request,
//     std::shared_ptr<GripperControl::Response> response)
//   {
//     if (!move_group_interface_) {
//       RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized.");
//       response->success = false;
//       response->message = "MoveGroupInterface not ready.";
//       return;
//     }

//     double target_position = request->close ? -0.5235 : 0.15;  // Updated closed/open positions
//     move_group_interface_->setJointValueTarget("robotiq_85_left_knuckle_joint", target_position);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//       RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing gripper movement...");

//       auto result = move_group_interface_->execute(plan);
//       move_group_interface_->stop();
//       rclcpp::sleep_for(std::chrono::milliseconds(300));

//       if (result == moveit::core::MoveItErrorCode::SUCCESS) {
//         RCLCPP_INFO(this->get_logger(), "Gripper moved to %.3f rad successfully.", target_position);
//         response->success = true;
//         response->message = "Gripper moved successfully.";
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Execution failed (code = %d).", result.val);
//         response->success = false;
//         response->message = "Gripper execution failed.";
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "Planning failed.");
//       response->success = false;
//       response->message = "Gripper planning failed.";
//     }
//   }

//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
//   rclcpp::Service<GripperControl>::SharedPtr gripper_service_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<GripperControlService>();
//   rclcpp::executors::SingleThreadedExecutor exec;
//   exec.add_node(node);
//   exec.spin();
//   rclcpp::shutdown();
//   return 0;
// }
