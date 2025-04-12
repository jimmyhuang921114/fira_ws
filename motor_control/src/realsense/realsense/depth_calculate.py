# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose
# from std_msgs.msg import String, Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class DepthToWorldNode(Node):
#     def __init__(self):
#         super().__init__('depth_to_world_node')
#         self.text_info_sub = self.create_subscription(String, '/ocr_results', self.text_info_callback, 10)
#         self.depth_sub = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
#         self.publisher = self.create_publisher(Pose, '/text_coordinate', 10)
#         self.bridge = CvBridge()

#         # RealSense 內参 (1280x720 解析度)
#         # RealSense 深度相機內參 (1280x720)
#         self.fx = 638.1703
#         self.fy = 638.1703
#         self.cx = 638.6412
#         self.cy = 355.8140

#         self.detected_texts = []
    
#     def text_info_callback(self, msg):
#         self.detected_texts = [item.split(',') for item in msg.data.split(';')]
    

    
#     def pixel_to_world(self, u, v, depth):
#         if depth <= 0 or np.isnan(depth) or np.isinf(depth):
#             return None  # 无效深度值
        
#         X = (u - self.cx) * depth / self.fx
#         Y =-( (v - self.cy) * depth / self.fy)
#         Z = depth
#         return (X, Y, Z)
    

#     def depth_callback(self, msg):
#         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
#         world_coordinates = []
#         for text, u, v in self.detected_texts:
#             u, v = int(u), int(v)
#             depth = depth_image[v, u] / 10000.0  # 假设深度单位是 mm 转换为米
#             world_coords = self.pixel_to_world(u, v, depth)
#             if world_coords:
#                 world_coordinates.append(f"{text},{world_coords[0]},{world_coords[1]},{world_coords[2]}")
        
#         if world_coordinates:
#             # 只發送第一筆座標
#             first_text, x, y, z = world_coordinates[0].split(',')
#             pose_msg = Pose()
#             pose_msg.position.x = float(x)
#             pose_msg.position.y = float(y)
#             pose_msg.position.z = float(z)
#             self.publisher.publish(pose_msg)
#             self.get_logger().info(f'Published World Coordinate for "{first_text}": ({x}, {y}, {z})')


# def main():
#     rclpy.init()
#     node = DepthToWorldNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

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

        # 訂閱 OCR 偵測到的文字與 pixel 座標
        self.text_sub = self.create_subscription(
            String, '/ocr_results', self.text_callback, 10)

        # 訂閱對齊後深度圖像
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10)

        # 訂閱彩色圖像（用於顯示畫面）
        self.color_sub = self.create_subscription(
            Image, '/camera/color', self.color_callback, 10)

        self.pose_pub = self.create_publisher(Pose, '/text_coordinate', 10)
        self.bridge = CvBridge()

        # 彩色相機 內參（需根據你的實機調整）
        self.fx = 919.0689
        self.fy = 919.3917
        self.cx = 623.3080
        self.cy = 369.0916

        self.detected_texts = []
        self.color_image = None

    def text_callback(self, msg):
        self.detected_texts = [item.split(',') for item in msg.data.split(';') if len(item.split(',')) == 3]

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        if self.color_image is None:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        vis_image = self.color_image.copy()

        for text, u_str, v_str in self.detected_texts:
            try:
                u, v = int(float(u_str)), int(float(v_str))
                if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
                    continue

                Z = depth_image[v, u] / 1000.0  # mm → m
                if Z <= 0 or np.isnan(Z) or np.isinf(Z):
                    continue

                X = (u - self.cx) * Z / self.fx
                Y =-( (v - self.cy) * Z / self.fy)

                # 發布 3D 座標
                pose = Pose()
                pose.position.x = float(X)
                pose.position.y = float(Y)
                pose.position.z = float(Z)
                self.pose_pub.publish(pose)

                # 顯示在 OpenCV 畫面上
                cv2.circle(vis_image, (u, v), 5, (0, 255, 0), -1)
                cv2.putText(vis_image, f'{text} ({X:.2f},{Y:.2f},{Z:.2f})', (u+10, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                self.get_logger().info(f'Text "{text}" at (u,v)=({u},{v}) -> (X,Y,Z)=({X:.2f}, {Y:.2f}, {Z:.2f})')

            except Exception as e:
                self.get_logger().warn(f'Error processing point: {e}')

        # 顯示視覺化結果
        cv2.imshow("Text Detection with 3D", vis_image)
        cv2.waitKey(1)

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
