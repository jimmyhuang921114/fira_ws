import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthToWorldNode(Node):
    def __init__(self):
        super().__init__('depth_to_world_node')
        self.text_info_sub = self.create_subscription(String, '/ocr_results', self.text_info_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.publisher = self.create_publisher(Pose, '/text_coordinate', 10)
        self.bridge = CvBridge()

        # RealSense 內参 (1280x720 解析度)
        # RealSense 深度相機內參 (1280x720)
        self.fx = 638.1703
        self.fy = 638.1703
        self.cx = 638.6412
        self.cy = 355.8140

        self.detected_texts = []
    
    def text_info_callback(self, msg):
        self.detected_texts = [item.split(',') for item in msg.data.split(';')]
    
    def pixel_to_world(self, u, v, depth):
        if depth <= 0 or np.isnan(depth) or np.isinf(depth):
            return None  # 无效深度值
        
        X = (u - self.cx) * depth / self.fx
        Y =-( (v - self.cy) * depth / self.fy)
        Z = depth
        return (X, Y, Z)
    
    
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        world_coordinates = []
        for text, u, v in self.detected_texts:
            u, v = int(u), int(v)
            depth = depth_image[v, u] / 1000.0  # 假设深度单位是 mm 转换为米
            world_coords = self.pixel_to_world(u, v, depth)
            if world_coords:
                world_coordinates.append(f"{text},{world_coords[0]},{world_coords[1]},{world_coords[2]}")
        
        if world_coordinates:
            # 只發送第一筆座標
            first_text, x, y, z = world_coordinates[0].split(',')
            pose_msg = Pose()
            pose_msg.position.x = float(x)
            pose_msg.position.y = float(y)
            pose_msg.position.z = float(z)
            self.publisher.publish(pose_msg)
            self.get_logger().info(f'Published World Coordinate for "{first_text}": ({x}, {y}, {z})')


def main():
    rclpy.init()
    node = DepthToWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
