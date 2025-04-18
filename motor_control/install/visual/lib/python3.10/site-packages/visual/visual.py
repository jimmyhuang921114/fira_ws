import sys, os
import rclpy, cv2, copy
import numpy as np

from launch_ros.substitutions import FindPackageShare

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QToolBar, QToolButton, QHBoxLayout, QSplitter, \
        QPushButton, QGraphicsPixmapItem, QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsItemGroup, \
        QGraphicsEllipseItem, QGraphicsRectItem, QApplication
from PySide6.QtGui import QImage, QPixmap, QIcon, QKeyEvent, QKeySequence, QColor, QBrush
from PySide6.QtCore import Qt, QTimer, QPoint, QRect, QRectF

from visual_ros import *


share_path = FindPackageShare(package = 'visual').find('visual') 
print(share_path)
image_path = os.path.join(share_path, "../../lib/visual")
print(image_path)


class RobotGraphicsItem(QGraphicsItemGroup):
    def __init__(self, size: float):
        super().__init__()
        ## dir rect
        dir_rect = QGraphicsRectItem(QRectF(0, -size * 0.2, size, size * 0.4))
        dir_rect.setBrush(QBrush(Qt.GlobalColor.red, Qt.BrushStyle.SolidPattern))
        self.addToGroup(dir_rect)
        ## out circle
        out_circle = QGraphicsEllipseItem(QRectF(-size*0.5, -size*0.5, size, size))
        out_circle.setBrush(QBrush(Qt.GlobalColor.blue, Qt.BrushStyle.SolidPattern))
        self.addToGroup(out_circle)


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 Map Viewer')
        self.setGeometry(100, 100, 800, 600)
        ## main layout
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)
        ## left layout
        left_layout = QVBoxLayout()
        main_layout.addLayout(left_layout)
        ## right layout
        right_layout = QVBoxLayout()
        right_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        main_layout.addLayout(right_layout)

        ## toolbar
        toolbar = QToolBar("toolbar")
        toolbar.setStyleSheet("QToolBar {border: 2px solid blue }")
        left_layout.addWidget(toolbar)
        ## move button
        self.move_btn = QToolButton()
        self.move_btn.setText("Move")
        self.move_btn.setCheckable(True)
        self.move_btn.clicked.connect(self.move_btn_clicked_event)
        toolbar.addWidget(self.move_btn)
        ## save button
        save_map_btn = QToolButton()
        save_map_btn.setText("Save")
        toolbar.addWidget(save_map_btn)
        ## load button
        load_map_btn = QToolButton()
        load_map_btn.setText("Load")
        toolbar.addWidget(load_map_btn)
        ## nav button
        self.nav_btn = QToolButton()
        self.nav_btn.setText("Nav")
        self.nav_btn.setCheckable(True)
        toolbar.addWidget(self.nav_btn)
        
        ## map scene
        self.map_scene = QGraphicsScene()
        self.map_view = QGraphicsView()
        self.map_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_view.setStyleSheet("QLabel {border: 1px solid red }")
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.map_view.verticalScrollBar().blockSignals(True)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.map_view.horizontalScrollBar().blockSignals(True)
        self.map_view.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.map_view.setScene(self.map_scene)
        left_layout.addWidget(self.map_view)
        ## map pixmap
        self.map_graphics_pixmap = self.map_scene.addPixmap(QPixmap())
        ## robot rect
        self.robot_graph_item = RobotGraphicsItem(5)
        self.map_scene.addItem(self.robot_graph_item)
        # self.robot_rect = self.map_scene.addRect(QRect(-5, -3, 10, 6), QColor("green"))
        ## map origin rect
        self.map_origin_rect = self.map_scene.addRect(QRect(-3, -3, 6, 6), QColor("red"))
        ## scene zero rect
        self.map_scene.addRect(QRect(0, 0, 5, 5), QColor("blue"))
        
        ## node launcher
        node_launcher_label = QLabel("Node Launcher")
        right_layout.addWidget(node_launcher_label)
        ## motor node button
        motor_node_btn = QPushButton("Motor")
        motor_node_btn.setCheckable(True)
        motor_node_btn.setFixedWidth(100)
        motor_node_btn.clicked.connect(self.motor_node_btn_clicked_event)
        right_layout.addWidget(motor_node_btn)
        ## lidar node button
        lidar_node_btn = QPushButton("Lidar")
        lidar_node_btn.setCheckable(True)
        lidar_node_btn.setFixedWidth(100)
        right_layout.addWidget(lidar_node_btn)
        ## map builder node button
        map_builder_node_btn = QPushButton("Builder")
        map_builder_node_btn.setCheckable(True)
        map_builder_node_btn.setFixedWidth(100)
        right_layout.addWidget(map_builder_node_btn)
        
        ## Variables
        self.ros_node: RosNode = None
        self.stop_gui = False
        self.pressed_keys = set()
        
        ## update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.setInterval(30)
        self.update_timer.start()


    def handle_pressed_keys(self):
        ## move button function
        if self.move_btn.isChecked():
            ## W no S
            if Qt.Key.Key_W in self.pressed_keys and not Qt.Key.Key_S in self.pressed_keys:
                self.ros_node.move_twist.linear.x = 0.1
            ## S no W
            elif Qt.Key.Key_S in self.pressed_keys and not Qt.Key.Key_W in self.pressed_keys:
                self.ros_node.move_twist.linear.x = -0.1
            ## other
            else:
                self.ros_node.move_twist.linear.x = 0.0
            ## A no D
            if Qt.Key.Key_A in self.pressed_keys and not Qt.Key.Key_D in self.pressed_keys:
                self.ros_node.move_twist.angular.z = 1.0
            ## D no A
            elif Qt.Key.Key_D in self.pressed_keys and not Qt.Key.Key_A in self.pressed_keys:
                self.ros_node.move_twist.angular.z = -1.0
            ## other
            else:
                self.ros_node.move_twist.angular.z = 0.0
            # print(self.move_twist)

    def fit_map_in_view(self):
        width = self.ros_node.map.info.width
        height = self.ros_node.map.info.height
        self.map_view.fitInView(QRect(0, 0, width, height), Qt.AspectRatioMode.KeepAspectRatio)
        scene_rect = self.map_view.sceneRect()
        scene_rect.moveLeft(-width*0.5)
        self.map_view.setSceneRect(scene_rect)

    def move_btn_clicked_event(self, event):
        self.ros_node.move_twist.linear.x = 0.0
        self.ros_node.move_twist.angular.z = 0.0
        ## enable/disable move output
        self.ros_node.move_enable(event)

    def motor_node_btn_clicked_event(self, event):
        # cmd = {
        #     'package'   : 'motor',
        #     'node'      : 'motor_odom'
        # }
        cmd = {
            'package'   : 'turtlesim',
            'node'      : 'turtlesim_node'
        }
        self.ros_node.send_cmd(cmd, event)

    def keyPressEvent(self, event: QKeyEvent):
        ## only on real key event
        if not event.isAutoRepeat():
            self.pressed_keys.add(event.key())
            self.handle_pressed_keys()
        return super().keyPressEvent(event) 

    def keyReleaseEvent(self, event: QKeyEvent):
        ## only on real key event
        if not event.isAutoRepeat():
            self.pressed_keys.remove(event.key())
            self.handle_pressed_keys()
        return super().keyReleaseEvent(event)

    def closeEvent(self, event):
        self.stop_gui = True 
        return super().closeEvent(event)

    def resizeEvent(self, event):
        self.fit_map_in_view
        return super().resizeEvent(event)

    def update(self):
        ## get map data
        map_msg = self.ros_node.map
        if map_msg is not None:
            width = map_msg.info.width
            height = map_msg.info.height
            data = map_msg.data
            frame = np.ones(shape=(500, 500), dtype=np.uint8) * 255
            # Convert occupancy grid data (-1: unknown, 0: free, 100: occupied) to grayscale
            for y in range(height):
                for x in range(width):
                    value = data[width * y + x]
                    if value == -1:
                        frame[height - 1 - y, x] = 0x80  # Unknown: mid-gray
                    elif value == 0:
                        frame[height - 1 - y, x] = 0xff  # Free: white
                    else:
                        frame[height - 1 - y, x] = 0x00  # Occupied: black
            frame = np.ascontiguousarray(frame)
            ## update map pixmap
            image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format.Format_Grayscale8)
            self.map_graphics_pixmap.setPixmap(QPixmap(image))
            self.fit_map_in_view()
            ## draw map origin
            self.map_origin_rect.setX(self.ros_node.map_origin.x)
            self.map_origin_rect.setY(self.ros_node.map_origin.y)
            # print(self.map_origin_rect.pos())
            ## get pos
            self.robot_graph_item.setX(self.ros_node.pose.position.x)
            self.robot_graph_item.setY(self.ros_node.pose.position.y)
            self.robot_graph_item.setRotation(-self.ros_node.pose.orientation.z)



def main():
    rclpy.init()
    app = QApplication(sys.argv)
    manager = GUI()
    ## ros node
    node = RosNode()
    node.create_rate(1000)
    ## start gui
    manager.ros_node = node
    manager.show()
    try:
        while True:
            # print(f"ret: {app.exec()}")
            app.processEvents()
            rclpy.spin_once(node)
            if manager.stop_gui:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        app.quit()


if __name__ == '__main__':
    main()
