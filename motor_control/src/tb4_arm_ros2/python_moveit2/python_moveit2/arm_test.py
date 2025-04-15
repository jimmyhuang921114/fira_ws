import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time
import yaml
import os
from python_moveit_interface.srv import PoseRequest, GripperControl, DetectPose, ArmControl

class ArmControlRouter(Node):
    def __init__(self):
        super().__init__('arm_control_router')

        self.auto_pose_client = self.create_client(DetectPose, 'auto_pose_service')
        self.named_pose_client = self.create_client(PoseRequest, 'named_pose_service')
        self.gripper_client = self.create_client(GripperControl, 'gripper_control_service')

        self.text_receiving = False
        self.latest_text_coordinate = ""
        self.text_subscriber = self.create_subscription(String, 'text_coordinate', self.text_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, 'target_pose_in_base', self.target_pose_callback, 10)
        self.latest_detect_pose = Pose()

        self.main_service = self.create_service(ArmControl, 'arm_control', self.run_task_service)

        config_path = os.path.join(os.path.dirname(__file__), 'task_config.yaml')
        with open(config_path, 'r') as f:
            self.task_config = yaml.safe_load(f)

        self.get_logger().info('✅ ArmControlRouter is ready.')

    def text_callback(self, msg):
        if self.text_receiving:
            self.get_logger().info(f"📦 Text coordinate received: {msg.data}")
            self.latest_text_coordinate = msg.data
        else:
            self.get_logger().warn("❌ Text coordinate not accepted (waiting flag off).")

    def target_pose_callback(self, msg):
        self.latest_detect_pose = msg

    def run_task_service(self, request, response):
        task = request.task_name.lower()
        try:
            if task in self.task_config:
                return self.run_task_sequence(self.task_config[task], response)
            else:
                response.success = False
                response.message = f"❌ Unknown task: {task}"
                return response
        except Exception as e:
            response.success = False
            response.message = f"❌ Exception: {e}"
            return response

    def run_task_sequence(self, steps, response):
        for step in steps:
            if 'name_pose' in step:
                self.send_named_pose(step['name_pose'], response)
            elif 'gripper' in step:
                close = step['gripper'] == 'close'
                self.send_gripper(close, response)
            elif 'detect_pose' in step:
                self.send_auto_pose(self.latest_detect_pose, response)
            elif 'log' in step:
                self.get_logger().info(f"📝 {step['log']}")
            elif 'wait' in step:
                time.sleep(float(step['wait']))
            else:
                self.get_logger().warn(f"⚠️ Unknown step: {step}")
        response.success = True
        response.message = "✔️ 任務完成"
        return response

    def send_named_pose(self, name, response):
        if not self.named_pose_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "named_pose_service not available"
            return response
        req = PoseRequest.Request()
        req.message = name
        future = self.named_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_auto_pose(self, pose, response):
        if not self.auto_pose_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "auto_pose_service not available"
            return response
        req = DetectPose.Request()
        req.target_pose = pose
        future = self.auto_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_gripper(self, close, response):
        if not self.gripper_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "gripper_control_service not available"
            return response
        req = GripperControl.Request()
        req.close = close
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = ArmControlRouter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()