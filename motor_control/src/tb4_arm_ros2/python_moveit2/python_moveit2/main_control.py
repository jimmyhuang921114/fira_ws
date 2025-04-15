from geometry_msgs.msg import Pose
from std.msgs.msg import String
from rclpy.node import Node
from python_moveit_interface.srv import ArmControl
import rclpy
import time
import os
import sys


class  MainControl(Node):
    def __init__(self):
        super().__init__("fira_main_contorl")
        
        #-----visiual node-----
        #color_detect_mode
        self.create_subscription(Pose,"/realsense/color_range",self.color_callback,10)
        #word_detect_mode
        self.create_subscription(list,"/realsense/depth_calculate",self.text_callback,10)
        
        #-----arm_control node-----
        self.arm_state_control=self.create_client(String,"/realsense/paddleorc",self.paddle_callback,10)
        
        #-----car_control node-----
        # self.car_state_client = self.create_client(PoseRequest, 'pose_goal_service')
    

    def color_callback(self,msg):
        self.get_logger().info("color_callback")
        self.color_sort = msg.color_sort

    def text_callback(self,msg):
        self.get_logger().info("text_callback")
        self.text_sort = msg.text_sort
        self.box_count = msg.box_count
        self.box_name = msg.box_name
        self.color = msg.color
        self.get_logger().info(f"box_count: {self.box_count}")
        self.get_logger().info(f"box_name: {self.box_name}")

    


def main(args=None):
    rclpy.init(args=args)
    main_control = MainControl()
    rclpy.spin(main_control)
    main_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()