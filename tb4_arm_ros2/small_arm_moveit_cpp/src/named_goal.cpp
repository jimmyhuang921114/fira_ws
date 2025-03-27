#include <memory>
#include <rclcpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class NamedGoalControl : public rclcpp::Node{
  public:
  NamedGoalControl() :Node("named_goal_controller")
  {
    move_group_interface_ = std::make_shard<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),"small_arm");
    goal_subscriber_ = this->create_subsciption<std_msgs::msg::String>(
      "named_goal_target",10,
      std::bind(&NamedGoalControl::goal_callback,this,std::placeholders::_1)
    );
    RCLCCP_INFO(this->get_logger(),"NameGoalControl ready:Wait for command");
  }
private:
  void goal_callback(const std_msg::msg::String::SharedPtr msg){
    std::string target = msg->data;
    RCLCPP(this-get_logger(),"Receive the target : %s",target.c_str())
    move_group_interface_->setNameTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success){
      RCLCPP_INFO(this->get_logger(),"Plan Success",target.c_str());
      move_group_interface_->execute(plan);
    }
    else{
      RCLCPP_ERROR(this->get_logger(),"Plan Fail",target.c_str());
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_subscriber_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_; 
}

int main(int argc,char * argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<NamedGoalControl>();
  rclcpp::spin();
  rclcpp::shutdown();
  return 0 ;

}