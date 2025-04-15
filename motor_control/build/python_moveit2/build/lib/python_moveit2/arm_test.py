import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool
from python_moveit_interface.srv import PoseRequest, GripperControl, DetectPose, ArmControl  # 假設三個子 service 已經定義

class ArmControlRouter(Node):
    def __init__(self):
        super().__init__('arm_control_router')

        # 建立子功能的 Client
        self.auto_pose_client = self.create_client(DetectPose, 'auto_pose_service')
        self.named_pose_client = self.create_client(PoseRequest, 'named_pose_service')
        self.gripper_client = self.create_client(GripperControl, 'gripper_control_service')

        # 建立接收主控的 service
        self.main_service = self.create_service(ArmControl, 'arm_control', self.handle_command)
        self.get_logger().info('✅ ArmControlRouter is ready.')

    def handle_command(self, request, response):
        try:
            if request.trigger_grab_flow:
                self.get_logger().info("🚀 Trigger grab flow")
                return self.send_named_pose("grab_pose_a", response)

            if request.trigger_place_flow:
                self.get_logger().info("📦 Trigger place flow")
                return self.send_named_pose("put_1", response)

            if request.named_pose:
                return self.send_named_pose(request.named_pose, response)

            if request.target_pose:
                return self.send_auto_pose(request.target_pose, response)

            return self.send_gripper(request.gripper_close, response)

        except Exception as e:
            response.success = False
            response.message = f"❌ Exception: {e}"
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
