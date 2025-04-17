import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time
import yaml
import os
from python_moveit_interface.srv import PoseRequest, GripperControl, DetectPose, ArmControl
from ament_index_python.packages import get_package_share_directory

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import  ReentrantCallbackGroup


class ArmControlRouter(Node):
    def __init__(self):
        super().__init__('arm_control_router')
        self.callback_group = ReentrantCallbackGroup()
        # === 建立 Service Client ===
        self.auto_pose_client = self.create_client(DetectPose, 'auto_pose_service',callback_group=self.callback_group)
        self.named_pose_client = self.create_client(PoseRequest, 'named_pose_service',callback_group=self.callback_group)
        self.gripper_client = self.create_client(GripperControl, 'gripper_control_service',callback_group=self.callback_group)

        # === 訂閱訊息（未啟用的狀態用於未來延伸）===
        self.text_receiving = False
        self.latest_text_coordinate = ""
        self.latest_detect_pose = Pose()
        self.text_subscriber = self.create_subscription(String, 'text_coordinate', self.text_callback, 10,callback_group=self.callback_group)
        self.pose_subscriber = self.create_subscription(Pose, 'target_pose_in_base', self.target_pose_callback, 10,callback_group=self.callback_group)

        # === 提供主要控制服務 ===
        self.main_service = self.create_service(ArmControl, 'arm_control', self.run_task_service,callback_group=self.callback_group)

        # === 讀取任務設定 YAML ===
        config_path = os.path.join(
            get_package_share_directory('python_moveit2'),  
            'config', 'task_config.yaml'
        )
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
        self.get_logger().info(f"📥 接收到任務：{task}")
        try:
            if task in self.task_config:
                self.run_task_sequence(self.task_config[task], response)
            else:
                response.success = False
                response.message = f"❌ Unknown task: {task}"
        except Exception as e:
            response.success = False
            response.message = f"❌ Exception: {str(e)}"
        return response  # ✅ 關鍵在這


    def run_task_sequence(self, steps, response):
        for i, step in enumerate(steps):
            self.get_logger().info(f"🔄 第 {i+1} 步：{step}")
            result = None

            if 'name_pose' in step:
                result = self.send_named_pose(step['name_pose'], response)
                if result and result.success:
                    self.get_logger().info(f"✅ Named pose '{step['name_pose']}' 執行完成")
            elif 'gripper' in step:
                close = step['gripper'] == 'close'
                result = self.send_gripper(close, response)
                if result and result.success:
                    self.get_logger().info(f"✅ Gripper {'關閉' if close else '開啟'} 完成")


            elif 'detect_pose' in step:
                result = self.send_auto_pose(self.latest_detect_pose, response)
                
                if result and result.success:
                    self.get_logger().info(f"✅ 自動偵測姿態動作完成")
            elif 'log' in step:
                self.get_logger().info(f"📝 {step['log']}")
                continue
            elif 'wait' in step:
                time.sleep(float(step['wait']))
                self.get_logger().info(f"⏳ 等待 {step['wait']} 秒完成")
                continue
            else:
                self.get_logger().warn(f"⚠️ 不明任務步驟: {step}")
                continue

            # ❌ 如果該步驟失敗
            if result is None or not result.success:
                self.get_logger().error(f"❌ 執行失敗: {result.message if result else 'No response'}")
                response.success = False
                response.message = f"中斷於第 {i+1} 步: {step}"
                return response

        response.success = True
        response.message = "✔️ 所有任務完成 OK"
        print("aaaa")
        return response


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
        # rclpy.spin_until_future_complete(self, future)
        
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
        while not future.done():
            time.sleep(0.1)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = ArmControlRouter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
