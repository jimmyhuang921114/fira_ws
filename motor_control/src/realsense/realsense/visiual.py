import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QHBoxLayout, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DualAreaViewer(QWidget, Node):
    def __init__(self):
        QWidget.__init__(self)
        Node.__init__(self, "dual_area_viewer_node")

        self.setWindowTitle("Left-Right Area with 4 Screens")
        self.setGeometry(100, 100, 1600, 900)

        self.bridge = CvBridge()

        # Layout
        main_layout = QHBoxLayout()
        left_widget = QWidget()
        left_widget.setStyleSheet("background-color: #dddddd;")
        main_layout.addWidget(left_widget)

        # 右邊四個螢幕
        self.labels = []
        right_layout = QGridLayout()
        for i in range(4):
            label = QLabel(f"Screen {i+1}")
            label.setFixedSize(480, 270)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("background-color: black; color: white; font-size: 20px;")
            self.labels.append(label)
            right_layout.addWidget(label, i // 2, i % 2)

        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        main_layout.addWidget(right_widget)

        main_layout.setStretch(0, 1)
        main_layout.setStretch(1, 1)
        self.setLayout(main_layout)

        # ROS2 訂閱四個 topic
        self.create_subscription(Image, '/camera/color', self.color_callback, 10)
        self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)
        self.create_subscription(Image, '/color_detect', self.color_detect_callback, 10)
        self.create_subscription(Image, '/word_detect', self.word_detect_callback, 10)

    def convert_and_display(self, img, label_idx):
        try:
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            h, w, ch = img.shape
            bytes_per_line = ch * w
            qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_img).scaled(
                self.labels[label_idx].size(), Qt.KeepAspectRatio
            )
            self.labels[label_idx].setPixmap(pixmap)
        except Exception as e:
            print(f"Display error: {e}")

    def color_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.convert_and_display(img, 0)

    def depth_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        img = img.astype('uint8')
        self.convert_and_display(img, 1)

    def color_detect_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.convert_and_display(img, 2)

    def word_detect_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.convert_and_display(img, 3)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    viewer = DualAreaViewer()

    # 用 PyQt5 的 QTimer 定期處理 ROS 事件
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(viewer, timeout_sec=0.01))
    timer.start(10)

    viewer.show()
    sys.exit(app.exec_())
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
