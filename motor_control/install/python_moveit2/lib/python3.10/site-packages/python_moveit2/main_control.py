from geometry_msgs.msg import Pose
from std_msgs.msg import String
from rclpy.node import Node
from python_moveit_interface.srv import ArmControl
# from car_interface.srv import PoseRequest
import rclpy
import time
import os
import sys


class  MainControl(Node):
    def __init__(self):
        super().__init__("fira_main_contorl")
        
        #-----visiual node-----
        #color_detect_mode
        self.create_subscription(String,"/color/sort",self.color_callback,10)
        #word_detect_mode
        self.create_subscription(Pose,"/text_coordinate",self.text_callback,10)
        
        #-----arm_control node-----
        self.arm_state_control=self.create_client(ArmControl,'/arm_statue_service',10)
        
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

    def send_arm_state(self, arm_state):
        if self.arm_wait:
            self.get_logger().info('Waiting for arm state...')

        elif self.arm_wait==False:
            self.get_logger().info('Sending arm state...')
            req = ArmControl.Request()
            req.message = arm_state
            self.arm_state_control.call_async(req).add_done_callback(self.arm_state_response_callback)
            self.get_logger().info(f"Arm state sent: {arm_state}")
        else:
            self.get_logger().error('Error')

    # def send_car_state(self, car_state):
    #     if self.car_wait:
    #         self.get_logger().info('Waiting for car state...')

    #     elif self.car_wait==False:
    #         self.get_logger().info('Sending car state...')
    #         req = PoseRequest.Request()
    #         req.message = car_state
    #         self.car_state_client.call_async(req).add_done_callback(self.car_state_response_callback)
    #         self.get_logger().info(f"Car state sent: {car_state}")
    #     else:
    #         self.get_logger().error('Error')


def main(args=None):
    rclpy.init(args=args)
    main_control = MainControl()
    rclpy.spin(main_control)
    main_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()