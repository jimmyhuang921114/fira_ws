#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class NextTriggerNode : public rclcpp::Node {
public:
  NextTriggerNode() : Node("next_trigger_node") {
    pub_ = this->create_publisher<std_msgs::msg::String>("/next_pose_trigger", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NextTriggerNode::publish_next, this));
    RCLCPP_INFO(this->get_logger(), "Press ENTER to publish 'next' command...");
  }

private:
  void publish_next() {
    std::string input;
    std::getline(std::cin, input);
    if (input == "next") {
      std_msgs::msg::String msg;
      msg.data = "next";
      pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published 'next' to /next_pose_trigger");
    } else {
      RCLCPP_INFO(this->get_logger(), "Type 'next' then ENTER to trigger movement");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NextTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
