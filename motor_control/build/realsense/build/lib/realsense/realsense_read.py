import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')

        # setting topic
        self.color_publisher = self.create_publisher(Image, '/camera/color', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)
        
        # opencv and ros2 tool
        self.bridge = CvBridge()

        # RealSense setup
        self.pipeline = None
        self.config = rs.config()

        # set timer
        self.timer = self.create_timer(0.01, self.publish_frames)

        # Connection status
        self.connected = False
        self.retry_count = 0
        self.max_retries = 5

        # Attempt to connect
        self.connect_camera()

    def connect_camera(self):
        self.get_logger().info("Attempting to connect to RealSense camera...")
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()

            config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 1280,720, rs.format.z16, 30)
            self.pipeline.start(config)
            self.connected = True
            self.retry_count = 0
            self.get_logger().info("Successfully connected to RealSense camera")

        except Exception as e:
            self.connected = False
            self.get_logger().error(f"Failed to connect to RealSense: {str(e)}")
            self.retry_reconnect()

    def retry_reconnect(self):
        if self.retry_count < self.max_retries:
            wait_time = 2 ** self.retry_count
            self.get_logger().warn(f"Retrying in {wait_time} seconds... ({self.retry_count+1}/{self.max_retries})")
            time.sleep(wait_time)
            self.retry_count += 1
            self.connect_camera()
        else:
            self.get_logger().error("Max retry attempts reached. Check the camera connection.")

    def publish_frames(self):
        if not self.connected:
            self.get_logger().warn("Camera not connected. Skipping.")
            return
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                self.get_logger().warn("Invalid frames. Skipping.")
                return
            

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')

            now = self.get_clock().now().to_msg()

            color_msg.header.stamp = now
            depth_msg.header.stamp = now

            self.color_publisher.publish(color_msg)
            self.depth_publisher.publish(depth_msg)

            self.get_logger().info("Published /camera/color and /camera/depth")
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Publishing error: {str(e)}")
            self.connected = False
            self.retry_reconnect()

    def destroy(self):
        if self.pipeline:
            self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt - stopping node.")
    finally:
        node.destroy()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
