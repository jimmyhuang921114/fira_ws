#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CameraToBasePrinter : public rclcpp::Node {
public:
  CameraToBasePrinter() 
  : Node("camera_to_base_printer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) 
  {
    // 在建構子中宣告參數（可自行調整）
    this->declare_parameter<std::vector<double>>(
      "cam_to_ee_translation", 
      {-0.053, 0.08, -0.6}  // <-- 這裡放你實測的平移
    );
    this->declare_parameter<std::vector<double>>(
      "cam_to_ee_quaternion", 
      {0.0, 0.0, 0.0, 1.0}   // <-- 這裡放對應的四元數(假設零旋轉)
    );

    // 建立訂閱者
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/world_coordinates", 
      10,
      std::bind(&CameraToBasePrinter::callback, this, std::placeholders::_1)
    );
  }

private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "❗ 接收到的 world_coordinates 長度不足 (< 3)");
      return;
    }

    // 1️⃣ 接收來自相機的點（假設是公尺）
    tf2::Vector3 point_in_camera(
      msg->data[0],
      msg->data[1],
      msg->data[2]
    );

    // 2️⃣ 讀取參數: camera → EE 的位置和四元數
    auto cam_to_ee_t = this->get_parameter("cam_to_ee_translation").as_double_array();
    auto cam_to_ee_q = this->get_parameter("cam_to_ee_quaternion").as_double_array();

    if (cam_to_ee_t.size() != 3 || cam_to_ee_q.size() != 4) {
      RCLCPP_WARN(this->get_logger(), "❌ 相機→末端參數長度不正確");
      return;
    }

    tf2::Vector3 translation_cam_to_ee(
      cam_to_ee_t[0],
      cam_to_ee_t[1],
      cam_to_ee_t[2]
    );
    tf2::Quaternion rotation_cam_to_ee(
      cam_to_ee_q[0],
      cam_to_ee_q[1],
      cam_to_ee_q[2],
      cam_to_ee_q[3]
    );

    // 建立相機→末端的 Transform
    tf2::Transform T_cam_to_ee(rotation_cam_to_ee, translation_cam_to_ee);

    try {
      // 3️⃣ 查詢 TF: link6 (EE) → base_link
      geometry_msgs::msg::TransformStamped ee_to_base_tf = tf_buffer_.lookupTransform(
        "base_link",  // target frame
        "link6",      // source frame
        tf2::TimePointZero
      );

      tf2::Transform T_ee_to_base;
      tf2::fromMsg(ee_to_base_tf.transform, T_ee_to_base);

      // 4️⃣ 相機點 → (相機→末端) → (末端→base)
      tf2::Vector3 point_in_ee   = T_cam_to_ee  * point_in_camera;
      tf2::Vector3 point_in_base = T_ee_to_base * point_in_ee;

      // 5️⃣ 印出結果
      RCLCPP_INFO(
        this->get_logger(),
        "📍 物體在 base_link 下的位置：x=%.3f, y=%.3f, z=%.3f",
        point_in_base.x(), 
        point_in_base.y(), 
        point_in_base.z()
      );

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "❌ 無法查詢 TF：%s", ex.what()
      );
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraToBasePrinter>());
  rclcpp::shutdown();
  return 0;
}
