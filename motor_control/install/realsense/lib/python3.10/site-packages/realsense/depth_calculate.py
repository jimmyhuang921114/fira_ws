# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import numpy as np
# import cv2
# import time 

# class RGBPixelTo3D(Node):
#     def __init__(self):
#         super().__init__('rgb_pixel_to_3d_node')

#         self.text_sub = self.create_subscription(String, '/ocr_results', self.text_callback, 10)
#         self.depth_sub = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
#         self.color_sub = self.create_subscription(Image, '/camera/color', self.color_callback, 10)

#         self.pose_pub = self.create_publisher(Pose, '/text_coordinate', 10)
#         self.bridge = CvBridge()

#         self.fx = 910.55
#         self.fy = 909.21
#         self.cx = 641.83
#         self.cy = 359.21
#         self.last_depth_time = time.time()
#         self.detected_texts = []
#         self.color_image = None

#     def text_callback(self, msg):
#         self.detected_texts = []
#         entries = [item.split(',') for item in msg.data.split(';') if len(item.split(',')) == 3]
#         for text, u_str, v_str in entries:
#             try:
#                 u, v = int(float(u_str)), int(float(v_str))
#                 self.detected_texts.append((text, u, v))
#             except ValueError:
#                 continue

#     def color_callback(self, msg):
#         self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     def fit_plane(self, points_3d):
#         A = np.c_[points_3d[:, 0], points_3d[:, 1], np.ones(points_3d.shape[0])]
#         B = points_3d[:, 2]
#         coeff, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
#         return coeff  # a, b, c

#     def depth_callback(self, msg):
#         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         region_size = 11
#         half_size = region_size // 2
#         failed_texts = []

#         for text, u, v in self.detected_texts:
#             if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
#                 failed_texts.append(text)
#                 continue

#             u_min = max(u - half_size, 0)
#             u_max = min(u + half_size + 1, depth_image.shape[1])
#             v_min = max(v - half_size, 0)
#             v_max = min(v + half_size + 1, depth_image.shape[0])

#             region = depth_image[v_min:v_max, u_min:u_max].astype(np.float32)
#             mask = region > 0
#             if np.count_nonzero(mask) < 10:
#                 failed_texts.append(text)
#                 continue

#             points = []
#             for vv in range(region.shape[0]):
#                 for uu in range(region.shape[1]):
#                     if mask[vv, uu]:
#                         du = u + uu - half_size
#                         dv = v + vv - half_size
#                         z = region[vv, uu] / 1000.0
#                         x = (du - self.cx) * z / self.fx
#                         y = -((dv - self.cy) * z / self.fy)
#                         points.append((x, y, z))

#             if len(points) < 10:
#                 failed_texts.append(text)
#                 continue

#             points_3d = np.array(points)
#             a, b, c = self.fit_plane(points_3d)
#             x_center = (u - self.cx) / self.fx
#             y_center = -((v - self.cy) / self.fy)
#             z_center = a * x_center + b * y_center + c

#             pose = Pose()
#             pose.position.x = x_center * z_center
#             pose.position.y = y_center * z_center
#             pose.position.z = float(z_center - 0.15)
#             self.pose_pub.publish(pose)

#             if self.color_image is not None:
#                 cv2.rectangle(self.color_image, (u_min, v_min), (u_max, v_max), (255, 0, 0), 1)
#                 cv2.circle(self.color_image, (u, v), 4, (0, 255, 0), -1)
#                 cv2.putText(
#                     self.color_image,
#                     f"{text}: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})",
#                     (u + 5, v - 5),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.5,
#                     (0, 255, 255),
#                     1
#                 )

#                 # 平面可視化 (以小網格畫出 3D 平面投影在影像上的線)
#                 for grid_u in range(u_min, u_max, 2):
#                     for grid_v in range(v_min, v_max, 2):
#                         grid_x = (grid_u - self.cx) / self.fx
#                         grid_y = -((grid_v - self.cy) / self.fy)
#                         grid_z = a * grid_x + b * grid_y + c
#                         # 繪製亮藍色點表面分布
#                         cv2.circle(self.color_image, (grid_u, grid_v), 1, (255, 255, 0), -1)

#             self.get_logger().info(
#                 f"Detected block '{text}' → (u,v)=({u},{v}) → (X,Y,Z)=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")

#         if self.color_image is not None:
#             cv2.imshow("Text Projection Debug", self.color_image)
#             cv2.waitKey(1)

#         if failed_texts:
#             self.get_logger().warn(f"Text(s) with no valid depth: {', '.join(failed_texts)}")

# def main():
#     rclpy.init()
#     node = RGBPixelTo3D()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()



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
        self.pose_pub = self.create_publisher(Pose, '/text_coordinate', 10)

        self.bridge = CvBridge()
        self.fx = 910.550048828125
        self.fy = 909.2122192382812
        self.cx = 641.8350219726562
        self.cy = 359.2066955566406

        self.detected_texts = []
        self.plane_params = None
        self.avg_z = None
        self.target_z = None

        cv2.namedWindow("Fitted Plane View", cv2.WINDOW_NORMAL)

    def text_callback(self, msg):
        self.detected_texts.clear()
        entries = [item.split(',') for item in msg.data.split(';') if len(item.split(',')) == 3]
        for text, u_str, v_str in entries:
            try:
                u, v = int(float(u_str)), int(float(v_str))
                self.detected_texts.append((text, u, v))
            except ValueError:
                continue

    def ransac_plane_fit(self, points, threshold=0.01, iterations=100):
        best_eq = None
        best_inliers = []
        for _ in range(iterations):
            sample = points[np.random.choice(points.shape[0], 3, replace=False)]
            p1, p2, p3 = sample
            normal = np.cross(p2 - p1, p3 - p1)
            if np.linalg.norm(normal) == 0:
                continue
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, p1)
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.where(distances < threshold)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_eq = (normal, d, best_inliers)
        return best_eq

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape
        box_size = 400
        u_start = w // 2 - box_size // 2
        v_start = h // 2 - box_size // 2

        points = []
        uvs = []
        region_depths = []

        for v in range(v_start, v_start + box_size, 2):
            for u in range(u_start, u_start + box_size, 2):
                z = depth_image[v, u] * 0.001
                if z == 0 or z > 2.0:
                    continue
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy
                points.append([x, y, z])
                uvs.append((u, v))
                region_depths.append(z)

        if len(points) < 10:
            self.get_logger().warn("Not enough points for plane fitting.")
            return

        self.avg_z = float(np.mean(region_depths))+0.025
        # self.avg_z =0.30
        self.get_logger().info(f"Average depth (avg_z): {self.avg_z:.3f} m")

        # Classify box height and assign target_z
        if 0.34 < self.avg_z < 0.38:
            # self.avg_z = 0.28   target_z
            self.target_z = 0.01
            self.get_logger().info("Height: LOW")
        elif 0.30 < self.avg_z < 0.34:
            # self.avg_z = 0.24
            self.target_z = 0.07
            self.get_logger().info("Height: MID")
        else:
            # self.avg_z = 0.22
            self.target_z = 0.11
            self.get_logger().info("Height: HIGH")

        result = self.ransac_plane_fit(np.array(points))
        if result is None:
            self.get_logger().warn("RANSAC plane fitting failed.")
            return

        normal, d, inlier_idx = result
        a, b, c = normal
        self.plane_params = (a, b, c, d)

        # --- Visualization ---
        vis = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        vis = cv2.cvtColor(vis.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        for idx in inlier_idx:
            u, v = uvs[idx]
            cv2.circle(vis, (u, v), 1, (0, 0, 255), -1)
        cv2.rectangle(vis, (u_start, v_start), (u_start + box_size, v_start + box_size), (255, 255, 0), 1)
        cv2.putText(vis, f"Avg Z: {self.avg_z:.3f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Fitted Plane View", vis)
        cv2.waitKey(10)

        # --- Publish 3D Pose of each OCR text ---
        for text, u, v in self.detected_texts:
            x = (u - self.cx) / self.fx
            y = -(v - self.cy) / self.fy
            z = self.avg_z
            pose = Pose()
            pose.position.x = x * z
            pose.position.y = y * z
            pose.position.z = self.target_z
            self.pose_pub.publish(pose)
            self.get_logger().info(
                f"OCR '{text}' @ ({u},{v}) → avgZ={z:.3f}m → (X,Y,Z)=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
            )

def main():
    rclpy.init()
    node = RGBPixelTo3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
