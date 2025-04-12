import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from your_package.srv import DetectPose, MoveToPose, GripperControl  # 替換為你的 package 名稱

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        self.detect_cli = self.create_client(DetectPose, 'detect_pose')
        self.move_cli = self.create_client(MoveToPose, 'move_to_pose')
        self.gripper_cli = self.create_client(GripperControl, 'gripper_control')

        self.timer = self.create_timer(1.0, self.run_sequence)

    def wait_for_service(self, client, name):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{name} service not available!')
            return False
        return True

    def run_sequence(self):
        self.timer.cancel()
        self.get_logger().info('🟢 Starting control sequence...')

        # 1️⃣ 呼叫 detect_pose
        if not self.wait_for_service(self.detect_cli, "detect_pose"):
            return

        detect_future = self.detect_cli.call_async(DetectPose.Request())
        rclpy.spin_until_future_complete(self, detect_future)
        if detect_future.result() is None:
            self.get_logger().error('❌ detect_pose failed')
            return

        target_pose = detect_future.result().target_pose
        self.get_logger().info('✅ Received target pose.')

        # 2️⃣ 呼叫 move_to_pose
        if not self.wait_for_service(self.move_cli, "move_to_pose"):
            return

        move_req = MoveToPose.Request()
        move_req.target_pose = target_pose

        move_future = self.move_cli.call_async(move_req)
        rclpy.spin_until_future_complete(self, move_future)
        if move_future.result() is None or not move_future.result().success:
            self.get_logger().error('❌ move_to_pose failed')
            return

        self.get_logger().info('✅ Move completed.')

        # 3️⃣ 呼叫 gripper_control（關閉）
        if not self.wait_for_service(self.gripper_cli, "gripper_control"):
            return

        grip_req = GripperControl.Request()
        grip_req.close = True

        grip_future = self.gripper_cli.call_async(grip_req)
        rclpy.spin_until_future_complete(self, grip_future)
        if grip_future.result() is None or not grip_future.result().success:
            self.get_logger().error('❌ gripper_control failed')
            return

        self.get_logger().info('✅ Gripper closed.')

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
