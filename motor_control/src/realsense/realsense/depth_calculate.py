import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class RGBPixelTo3D(Node):
    def __init__(self):
        super().__init__('rgb_pixel_to_3d_node')

        self.text_sub = self.create_subscription(String, '/ocr_results', self.text_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.color_sub = self.create_subscription(Image, '/camera/color', self.color_callback, 10)

        self.pose_pub = self.create_publisher(Pose, '/text_coordinate', 10)
        self.bridge = CvBridge()

        self.fx = 910.55
        self.fy = 909.21
        self.cx = 641.83
        self.cy = 359.21

        self.detected_texts = []  # [(text, u, v)]
        self.color_image = None

    def text_callback(self, msg):
        self.detected_texts = []
        entries = [item.split(',') for item in msg.data.split(';') if len(item.split(',')) == 3]
        for text, u_str, v_str in entries:
            try:
                u, v = int(float(u_str)), int(float(v_str))
                self.detected_texts.append((text, u, v))
            except ValueError:
                continue

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        region_size = 11
        half_size = region_size // 2
        failed_texts = []

        for text, u, v in self.detected_texts:
            if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
                failed_texts.append(text)
                continue

            u_min = max(u - half_size, 0)
            u_max = min(u + half_size + 1, depth_image.shape[1])
            v_min = max(v - half_size, 0)
            v_max = min(v + half_size + 1, depth_image.shape[0])

            region = depth_image[v_min:v_max, u_min:u_max].astype(np.float32)
            region = region[region > 0]
            if region.size < 10:
                failed_texts.append(text)
                continue

            Z = np.median(region) / 1000.0
            X = (u - self.cx) * Z / self.fx
            Y = -((v - self.cy) * Z / self.fy)

            pose = Pose()
            pose.position.x = float(X)
            pose.position.y = float(Y)
            pose.position.z = float((Z-0.15))
            self.pose_pub.publish(pose)

            if self.color_image is not None:
                cv2.rectangle(self.color_image, (u_min, v_min), (u_max, v_max), (255, 0, 0), 1)
                cv2.circle(self.color_image, (u, v), 4, (0, 255, 0), -1)
                cv2.putText(
                    self.color_image,
                    f"{text}: ({X:.2f}, {Y:.2f}, {Z:.2f})",
                    (u + 5, v - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )

            self.get_logger().info(
                f"Detected block '{text}' → (u,v)=({u},{v}) → (X,Y,Z)=({X:.3f}, {Y:.3f}, {Z:.3f})")

        if self.color_image is not None:
            cv2.imshow("Text Projection Debug", self.color_image)
            cv2.waitKey(1)

        if failed_texts:
            self.get_logger().warn(f"Text(s) with no valid depth: {', '.join(failed_texts)}")

def main():
    rclpy.init()
    node = RGBPixelTo3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()