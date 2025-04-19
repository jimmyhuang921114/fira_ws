#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
    // 宣告相機到 EE 的位移與旋轉參數
    this->declare_parameter<std::vector<double>>("camera_to_ee_translate", {-0.04, 0.04, -0.065});
    this->declare_parameter<std::vector<double>>("camera_to_ee_quaternion", {0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<bool>("enable_z_scale_div10", true);

    // 訂閱相機座標的 Pose 資料（通常為文字座標）
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/text_coordinate", 10,
      std::bind(&CameraToBasePrinter::callback, this, std::placeholders::_1)
    );

    // 發布轉換到 base_link 座標下的目標 Pose
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose_in_base", 10);
  }

private:
  void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    geometry_msgs::msg::Pose input_pose = *msg;
    RCLCPP_INFO(this->get_logger(), "text_callback");

    // 如果啟用 Z 縮放，將 Z 從 mm 除以 10（避免輸入單位錯誤）
    bool enable_div10 = this->get_parameter("enable_z_scale_div10").as_bool();
    if (enable_div10) {
      double original_z = input_pose.position.z;
      input_pose.position.z /= 10.0;
      RCLCPP_INFO(this->get_logger(), "Z-scaling enabled: original Z=%.4f → scaled Z=%.4f", original_z, input_pose.position.z);
    }

    // 將輸入 Pose 轉為 tf2 的 Vector 與 Quaternion 格式
    tf2::Vector3 point_in_camera(input_pose.position.x, input_pose.position.y, input_pose.position.z);
    tf2::Quaternion rotation_in_camera(
      input_pose.orientation.x,
      input_pose.orientation.y,
      input_pose.orientation.z,
      input_pose.orientation.w
    );

    // 讀取相機 → EE 的靜態轉換參數
    auto cam_to_ee_t = this->get_parameter("camera_to_ee_translate").as_double_array();
    auto cam_to_ee_q = this->get_parameter("camera_to_ee_quaternion").as_double_array();

    if (cam_to_ee_t.size() != 3 || cam_to_ee_q.size() != 4) {
      RCLCPP_WARN(this->get_logger(), "Parameter format incorrect: camera_to_ee");
      return;
    }

    // 建立 T_cam_to_ee 轉換
    tf2::Vector3 translation_cam_to_ee(cam_to_ee_t[0], cam_to_ee_t[1], cam_to_ee_t[2]);
    tf2::Quaternion rotation_cam_to_ee(cam_to_ee_q[0], cam_to_ee_q[1], cam_to_ee_q[2], cam_to_ee_q[3]);
    tf2::Transform T_cam_to_ee(rotation_cam_to_ee, translation_cam_to_ee);

    try {
      // 取得 TF: link6 → base_link
      geometry_msgs::msg::TransformStamped ee_to_base_tf =
        tf_buffer_.lookupTransform("base_link", "link6", tf2::TimePointZero);

      tf2::Transform T_ee_to_base;
      tf2::fromMsg(ee_to_base_tf.transform, T_ee_to_base);

      // 將點從 camera → ee → base_link
      tf2::Vector3 point_in_ee = T_cam_to_ee * point_in_camera;
      tf2::Vector3 point_in_base = T_ee_to_base * point_in_ee;

      // 旋轉部分同樣進行轉換
      tf2::Quaternion rotation_in_ee = rotation_cam_to_ee * rotation_in_camera;
      tf2::Quaternion rotation_in_base = T_ee_to_base.getRotation() * rotation_in_ee;

      // 將結果打包並發布
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = point_in_base.x();
      target_pose.position.y = point_in_base.y();
      target_pose.position.z = point_in_base.z();
      target_pose.orientation = tf2::toMsg(rotation_in_base);

      publisher_->publish(target_pose);

      RCLCPP_INFO(this->get_logger(),
        "Transformed Pose in base_link:\n  Position: x=%.3f y=%.3f z=%.3f\n  Orientation: x=%.3f y=%.3f z=%.3f w=%.3f",
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w
      );

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraToBasePrinter>());
  rclcpp::shutdown();
  return 0;
}
