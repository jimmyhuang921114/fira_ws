import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from python_moveit_interface.srv import ArmControl
# from map_if.srv import Command
import yaml
import os
import time

class MainControl(Node):
    def __init__(self):
        super().__init__('fira_main_control')

        self.create_subscription(String, "/color/sort", self.color_callback, 10)
        self.create_subscription(Pose, "/text_coordinate", self.text_callback, 10)

        self.arm_state_control = self.create_client(ArmControl, '/arm_statue_service')
        while not self.arm_state_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ArmControl service...')

        self.arm_task_sequence = [
            "initial_pose",
            "detect_pose",
            "grab_detect_pose_left_up",
            "put_left_up_block"
        ]

        self.car_task_sequence = [
            # e.g. ['home', 'left'], ['left', 'top'], ... 可擴充
        ]

        self.execute_task_sequence()

    def execute_task_sequence(self):
        self.get_logger().info("[START] Executing full arm task sequence")
        for task_name in self.arm_task_sequence:
            self.get_logger().info(f"[TASK] Sending arm command: {task_name}")
            success = self.send_arm_state(task_name)
            if not success:
                self.get_logger().error(f"[ABORT] Task failed: {task_name}")
                break
            time.sleep(1.0) 
        self.get_logger().info("[DONE] Arm task sequence completed")

    def send_arm_state(self, command):
        req = ArmControl.Request()
        req.message = command
        future = self.arm_state_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"[SUCCESS] Arm state sent: {command}")
            return True
        else:
            self.get_logger().error(f"[FAIL] Failed to send arm state: {command}")
            return False

    def color_callback(self, msg):
        self.get_logger().info(f"[COLOR] Detected: {msg.data}")

    def text_callback(self, msg):
        self.get_logger().info(f"[TEXT] Pose received: x={msg.position.x:.2f}, y={msg.position.y:.2f}, z={msg.position.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = MainControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
