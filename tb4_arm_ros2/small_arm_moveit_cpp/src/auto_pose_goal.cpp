#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msg/msg/string.hpp>
#include <geometry_msgs/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PoseGoalNode : public rclcpp::Node
{
public:
    PoseGoalNode():Node("pose_goal_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("pose_goal_stauts",10)
        subscriber_= this->create_subsciption<std_msgs::msg::Pose>("pose_goal_position",10,
        std::bind(&PoseGoalNode::cmdCallback,this,std::placeholders::_1)//
        )
    }
logger_=this->get_logger();
move_group_interface_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(),"small_arm");
RCLPP_INFO(logger_,"PoseGoalNode");


private:
    void cmdCallback(const geometry_msgs::msg:String::SharedPtr msg)
    {
        RCLCPP_INFO(logger_,"{SUB}recevie the command:'%s",msg->data.c_str());
        geometry_msg::msg::Pose target_pose = *msg
        move_group_interface_->setPoseTarget(target_pose);//givethe arm target position

        moveit::planning_interface::MoveGroupIneterface::Plan my_plan;
        bool success = (move_group_interface_->plan(my_plan) == moveit::core::<MoveItErrorCode::SUCCESS);

    if(success){
        RCLCPP_INFO(logger_,"{Moveit}Plan Success");
        move_group_interface_->execute(my_plan);
        std_msgs::msg;:String status_msg;
        status_msg.data ="{PUB}Work Success";
        publisher_->publish(status_msg);
    }
    else{
        RCLCPP_ERROR(logger_,"[Moveit]Plan Fail")
        std_msgs::msg::String status_msg;
        status_msg.data = "{PUB}Plan Fail";
        publisher_->publish(status_msg);
    }
}

rclcpp::Publisher<std_msgs::msg::String>::ShardPtr publisher_;
rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subsriber_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
rclcpp::Logger logger_;
}

int main(int argc,char * argv[])
{
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<PoseGoalNode>());
 rclcpp::shutdown();
 return 0;   
}