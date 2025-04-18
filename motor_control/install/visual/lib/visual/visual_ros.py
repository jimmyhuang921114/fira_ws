import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Transform, Pose, Point
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler
import math as m
import json


class RosNode(Node):
    def __init__(self):
        super().__init__('GUI_Node')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            2
        )
        self.move_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            2
        )
        self.cmd_pub = self.create_publisher(
            String,
            '/command',
            2
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.loop_timer = self.create_timer(0.03, self.loop_callback)
        ## Variable
        self.move_twist: Twist = Twist()
        self.map: OccupancyGrid = None
        self.map_origin: Point = Point() # in pixel
        self.pose: Pose = Pose() # in pixel and angle rad


    def map_callback(self, msg: OccupancyGrid):
        ## receive map
        self.map = msg
        self.map_origin.x = -msg.info.origin.position.x / msg.info.resolution
        self.map_origin.y = msg.info.height + msg.info.origin.position.y / msg.info.resolution

    def loop_callback(self):
        ## update pose tf
        pose_tf = None
        try:
            pose_tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()).transform
            # print(pose_tf)
        except:
            pass
        if pose_tf is not None  \
                and self.map is not None \
                and self.map.info.resolution:
            ## get rotation from pose_tf
            quaternion = [pose_tf.rotation.w, pose_tf.rotation.x, pose_tf.rotation.y, pose_tf.rotation.z]
            roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')
            ## update pose
            # print(f"x: {pose_tf.translation.x}, y: {pose_tf.translation.y}")
            self.pose.position.x = pose_tf.translation.x / self.map.info.resolution + self.map_origin.x
            self.pose.position.y = -pose_tf.translation.y / self.map.info.resolution + self.map_origin.y
            self.pose.orientation.z = yaw*180/m.pi
            # print(f"x:{self.pose.position.x}, y:{self.pose.position.y}")
            # print(f"yaw:{self.pose.orientation.z}")

    def move_enable(self, en: bool):
        if en:
            self.move_pub_timer = self.create_timer(0.01, self.send_move)
        else:
            self.move_pub_timer.destroy()
            self.move_pub.publish(Twist())

    def send_move(self):
        self.move_pub.publish(self.move_twist)
        # print(self.gui.move_twist)

    def send_cmd(self, cmd: dict, enable: bool):
        cmd['enable'] = enable
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
