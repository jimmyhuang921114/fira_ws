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
    this->declare_parameter<std::vector<double>>("camera_to_ee_translate", {-0.04, 0.035,-0.065});
    this->declare_parameter<std::vector<double>>("camera_to_ee_quaternion", {0.0, 0.0, 0.0, 1.0});
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/text_coordinate", 10,
      std::bind(&CameraToBasePrinter::callback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose_in_base", 10);
  }

private:
  void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    geometry_msgs::msg::Pose input_pose = *msg;

    tf2::Vector3 point_in_camera(input_pose.position.x, input_pose.position.y, 0.0);  // z 不參與變換
    tf2::Quaternion rotation_in_camera(
      input_pose.orientation.x,
      input_pose.orientation.y,
      input_pose.orientation.z,
      input_pose.orientation.w
    );

    auto cam_to_ee_t = this->get_parameter("camera_to_ee_translate").as_double_array();
    auto cam_to_ee_q = this->get_parameter("camera_to_ee_quaternion").as_double_array();

    if (cam_to_ee_t.size() != 3 || cam_to_ee_q.size() != 4) {
      RCLCPP_WARN(this->get_logger(), "Parameter format incorrect: camera_to_ee");
      return;
    }

    tf2::Vector3 translation_cam_to_ee(cam_to_ee_t[0], cam_to_ee_t[1], cam_to_ee_t[2]);
    tf2::Quaternion rotation_cam_to_ee(cam_to_ee_q[0], cam_to_ee_q[1], cam_to_ee_q[2], cam_to_ee_q[3]);
    tf2::Transform T_cam_to_ee(rotation_cam_to_ee, translation_cam_to_ee);

    try {
      geometry_msgs::msg::TransformStamped ee_to_base_tf =
        tf_buffer_.lookupTransform("base_link", "link6", tf2::TimePointZero);

      tf2::Transform T_ee_to_base;
      tf2::fromMsg(ee_to_base_tf.transform, T_ee_to_base);

      tf2::Vector3 point_in_ee = T_cam_to_ee * point_in_camera;
      tf2::Vector3 camera_to_base = T_ee_to_base * T_cam_to_ee * tf2::Vector3(0, 0, 0);
      tf2::Vector3 point_in_base = T_ee_to_base * point_in_ee;
      tf2::Vector3 ee_in_base = T_ee_to_base * tf2::Vector3(0, 0, 0);

      tf2::Vector3 translation = T_ee_to_base.getOrigin();

      RCLCPP_INFO(this->get_logger(),
      "ee_in_base: x=%.3f y=%.3f z=%.3f",
      ee_in_base.x(), ee_in_base.y(), ee_in_base  .z());
      RCLCPP_INFO(this->get_logger(),
      "Camera to base: x=%.3f y=%.3f z=%.3f",
      camera_to_base.x(), camera_to_base.y(), camera_to_base.z());
      // Construct output pose
      geometry_msgs::msg::Pose target_pose;
      double dx = -0.04;  // 向 +X 修正 5mm
      double dy = 0.035; // 向 -Y 修正 3mm
      target_pose.position.x = point_in_base.x() + dx;
      target_pose.position.y = point_in_base.y() + dy;
      target_pose.position.z = input_pose.position.z;

      tf2::Quaternion ee_current_rotation = T_ee_to_base.getRotation();
      target_pose.orientation = tf2::toMsg(ee_current_rotation);
      std::cout << "Orientation: "
          << "x=" << target_pose.orientation.x << " "
          << "y=" << target_pose.orientation.y << " "
          << "z=" << target_pose.orientation.z << " "
          << "w=" << target_pose.orientation.w << std::endl;

      RCLCPP_DEBUG(this->get_logger(),
        "Orientation: x=%.3f y=%.3f z=%.3f w=%.3f",
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
        publisher_->publish(target_pose);

      RCLCPP_INFO(this->get_logger(),
        "Converted Pose:\n  XY Transformed | Z (raw+offset)\n  → x=%.3f y=%.3f z=%.3f",
        target_pose.position.x, target_pose.position.y, target_pose.position.z
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
