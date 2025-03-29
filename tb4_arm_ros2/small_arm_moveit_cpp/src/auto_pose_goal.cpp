#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PoseGoalNode : public rclcpp::Node
{
public:
    PoseGoalNode()
    : Node("pose_goal_node"),
      logger_(this->get_logger())
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("pose_goal_status", 10);

        subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose_goal_position", 10,
            std::bind(&PoseGoalNode::cmdCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(logger_, "PoseGoalNode started.");
    }

private:
    void cmdCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!move_group_interface_)
        {
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "small_arm");
            move_group_interface_->setPlanningTime(5.0);  // Optional: Set a max planning time
        }

        RCLCPP_INFO(logger_, "[SUB] Received target pose: [x=%.4f, y=%.4f, z=%.4f]",
                    msg->position.x, msg->position.y, msg->position.z);

        move_group_interface_->setPoseTarget(*msg);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
        std_msgs::msg::String status_msg;
        if (success)
        {
            RCLCPP_INFO(logger_, "[MoveIt] Plan success. Executing...");
            move_group_interface_->execute(my_plan);
            status_msg.data = "[PUB] Work success.";
        }
        else
        {
            RCLCPP_ERROR(logger_, "[MoveIt] Plan failed.");
            status_msg.data = "[PUB] Plan failed.";
        }

        publisher_->publish(status_msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Logger logger_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseGoalNode>());
    rclcpp::shutdown();
    return 0;
}
