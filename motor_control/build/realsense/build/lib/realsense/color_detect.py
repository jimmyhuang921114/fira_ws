import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose

class ColorObjectTracker(Node):
    def __init__(self):
        super().__init__('color_object_tracker')
        self.bridge = CvBridge()

        # 訂閱 Realsense 彩色影像
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)

        # 發布抓到的位置
        self.pose_pub = self.create_publisher(Pose, '/color_object_pose', 10)

        # HSV 色彩範圍（黃、綠、藍）
        self.color_ranges = {
            'yellow': ((20, 100, 100), (30, 255, 255)),
            'green': ((40, 40, 40), (80, 255, 255)),
            'blue': ((100, 150, 0), (140, 255, 255))
        }

    def color_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color_name, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx, cy = x + w // 2, y + h // 2

                    pose = Pose()
                    pose.position.x = float(cx)

                    self.pose_pub.publish(pose)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    cv2.putText(frame, color_name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Tracked Colors", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
