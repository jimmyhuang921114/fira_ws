import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time
import yaml
import os
from python_moveit_interface.srv import PoseRequest, GripperControl, DetectPose, ArmControl
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class ArmControlRouter(Node):
    def __init__(self):
        super().__init__('arm_control_router')

        # 設定 callback group（可支援多執行緒）
        self.callback_group = ReentrantCallbackGroup()

        # 建立三個 service client
        self.auto_pose_client = self.create_client(DetectPose, 'auto_pose_service', callback_group=self.callback_group)
        self.named_pose_client = self.create_client(PoseRequest, 'named_pose_service', callback_group=self.callback_group)
        self.gripper_client = self.create_client(GripperControl, 'gripper_control_service', callback_group=self.callback_group)

        # 訂閱來自 vision 模組的目標 pose 資訊
        self.text_receiving = True
        self.latest_text_coordinate = ""
        self.latest_detect_pose = Pose()

        self.text_subscriber = self.create_subscription(Pose, 'text_coordinate', self.text_callback, 10, callback_group=self.callback_group)
        self.pose_subscriber = self.create_subscription(Pose, 'target_pose_in_base', self.target_pose_callback, 10, callback_group=self.callback_group)

        # 建立主要任務控制服務
        self.main_service = self.create_service(ArmControl, 'arm_control', self.run_task_service, callback_group=self.callback_group)

        # 載入任務對應表（task_config.yaml）
        config_path = os.path.join(get_package_share_directory('python_moveit2'), 'config', 'task_config.yaml')
        with open(config_path, 'r') as f:
            self.task_config = yaml.safe_load(f)

        self.get_logger().info('ArmControlRouter initialized.')

    # 訂閱 text_coordinate 回調函式
    def text_callback(self, msg):
        if self.text_receiving:
            self.get_logger().info(
                f"Text coordinate received: position=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})"
            )
            self.latest_text_coordinate = msg
        else:
            self.get_logger().warn("Text coordinate not accepted (flag off).")

    # 訂閱 target_pose_in_base 回調函式
    def target_pose_callback(self, msg):
        self.latest_detect_pose = msg

    # 主任務服務：執行任務流程
    def run_task_service(self, request, response):
        task = request.task_name.lower()
        self.get_logger().info(f"Received task: {task}")
        try:
            if task in self.task_config:
                self.run_task_sequence(self.task_config[task], response)
            else:
                response.success = False
                response.message = f"Unknown task: {task}"
        except Exception as e:
            response.success = False
            response.message = f"Exception: {str(e)}"
        return response

    # 執行任務中的每個步驟
    def run_task_sequence(self, steps, response):
        for i, step in enumerate(steps):
            self.get_logger().info(f"Step {i+1}: {step}")
            result = None

            # 處理 named_pose 動作
            if 'name_pose' in step:
                result = self.send_named_pose(step['name_pose'], response)
                if result and result.success:
                    self.get_logger().info(f"Named pose '{step['name_pose']}' completed.")

            # 處理 gripper 動作
            elif 'gripper' in step:
                close = step['gripper'] == 'close'
                result = self.send_gripper(close, response)
                if result and result.success:
                    self.get_logger().info(f"Gripper {'closed' if close else 'opened'} successfully.")

            # 處理自動姿態偵測（整合 MoveIt 追蹤）
            elif 'detect_pose' in step:
                if step['detect_pose']:
                    result = self.send_auto_pose(response)
                    if result and result.success:
                        self.get_logger().info("Auto pose detection completed.")
                else:
                    self.get_logger().warn("'detect_pose' is False or None. Skipped.")

            # 顯示訊息
            elif 'log' in step:
                self.get_logger().info(f"{step['log']}")
                continue

            # 等待時間
            elif 'wait' in step:
                time.sleep(float(step['wait']))
                self.get_logger().info(f"Waited {step['wait']} seconds.")
                continue

            # 其他未知指令
            else:
                self.get_logger().warn(f"Unknown step: {step}")
                continue

            # 若步驟執行失敗，終止整個任務
            if result is None or not result.success:
                self.get_logger().error(f"Step failed: {result.message if result else 'No response'}")
                response.success = False
                response.message = f"Aborted at step {i+1}: {step}"
                return response

        # 任務流程完成
        response.success = True
        response.message = "All steps completed successfully."
        print("Task sequence finished.")
        return response

    # 呼叫 named_pose_service
    def send_named_pose(self, name, response):
        if not self.named_pose_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "named_pose_service not available"
            return response
        req = PoseRequest.Request()
        req.message = name
        future = self.named_pose_client.call_async(req)
        while not future.done():
            time.sleep(0.1)
        return future.result()

    # 呼叫 auto_pose_service
    def send_auto_pose(self, response):
        if not self.auto_pose_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "auto_pose_service not available"
            return response
        req = DetectPose.Request()
        future = self.auto_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # 呼叫 gripper_control_service
    def send_gripper(self, close, response):
        if not self.gripper_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "gripper_control_service not available"
            return response
        req = GripperControl.Request()
        req.close = close
        future = self.gripper_client.call_async(req)
        while not future.done():
            time.sleep(0.1)
        return future.result()

# 節點啟動主程式
def main():
    rclpy.init()
    node = ArmControlRouter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
