from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray, Int8, String, Float32
import numpy as np
from geometry_msgs.msg import Pose

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test_node')
        self.sub