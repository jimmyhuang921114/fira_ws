#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

class CameraToBasePrinter : public rclcpp::Node {
public:
  CameraToBasePrinter()
  : Node("camera_to_base_printer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // tranform parameter
    this->declare_parameter<std::vector<double>>("camera_to_ee_translate", {-0.053, 0.05, -0.6});
    this->declare_parameter<std::vector<double>>("camera_to_ee_quaternion", {0.0, 0.0, 0.0, 1.0});

    // sub the positionn from camera
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("/text_coordinate", 10,std::bind(&CameraToBasePrinter::callback, this, std::placeholders::_1)
    );

    // 建立 publisher，發佈轉換後的 PoseStamped
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_in_base", 10);
  }

private:
  void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // 將 Pose 中的位置轉成 tf2::Vector3
    tf2::Vector3 point_in_camera(msg->position.x,msg->position.y,msg->position.z);

    // 取得 camera → EE 的轉換參數
    auto cam_to_ee_t = this->get_parameter("camera_to_ee_translate").as_double_array();
    auto cam_to_ee_q = this->get_parameter("camera_to_ee_quaternion").as_double_array();

    if (cam_to_ee_t.size() != 3 || cam_to_ee_q.size() != 4) {RCLCPP_WARN(this->get_logger(), "❌ camera_to_ee parameter wrong");
      return;
    }

    tf2::Vector3 translation_cam_to_ee(cam_to_ee_t[0], cam_to_ee_t[1], cam_to_ee_t[2]);
    tf2::Quaternion rotation_cam_to_ee(cam_to_ee_q[0], cam_to_ee_q[1], cam_to_ee_q[2], cam_to_ee_q[3]);
    tf2::Transform T_cam_to_ee(rotation_cam_to_ee, translation_cam_to_ee);

    try {
      // 查詢 TF：末端執行器 → 機器人底座
      geometry_msgs::msg::TransformStamped ee_to_base_tf = tf_buffer_.lookupTransform("base_link", "link6", tf2::TimePointZero);

      tf2::Transform T_ee_to_base;
      tf2::fromMsg(ee_to_base_tf.transform, T_ee_to_base);

      // 座標轉換
      tf2::Vector3 point_in_ee = T_cam_to_ee * point_in_camera;
      tf2::Vector3 point_in_base = T_ee_to_base * point_in_ee;

      // 建立 PoseStamped 訊息
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.stamp = this->now();
      target_pose.header.frame_id = "base_link";
      target_pose.pose.position.x = point_in_base.x();
      target_pose.pose.position.y = point_in_base.y();
      target_pose.pose.position.z = point_in_base.z();
      target_pose.pose.orientation.w = 1.0;  // 無旋轉（如需轉向可加入計算）

      // 發佈
      publisher_->publish(target_pose);

      // 印出訊息
      RCLCPP_INFO(this->get_logger(),
        "Publish base_link coordinate:x=%.3f y=%.3f z=%.3f",
        point_in_base.x(), point_in_base.y(), point_in_base.z());

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF translate fail%s", ex.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraToBasePrinter>());
  rclcpp::shutdown();
  return 0;
}
