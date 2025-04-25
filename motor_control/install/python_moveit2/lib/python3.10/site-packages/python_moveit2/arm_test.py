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

        self.callback_group = ReentrantCallbackGroup()
        self.cartesian_client = self.create_client(DetectPose, 'compute_cartesian_path', callback_group=self.callback_group)

        self.auto_pose_client = self.create_client(DetectPose, 'auto_pose_service', callback_group=self.callback_group)
        self.named_pose_client = self.create_client(PoseRequest, 'named_pose_service', callback_group=self.callback_group)
        self.gripper_client = self.create_client(GripperControl, 'gripper_control_service', callback_group=self.callback_group)

        self.text_receiving = True
        self.latest_text_coordinate = ""
        self.latest_detect_pose = Pose()

        # self.text_subscriber = self.create_subscription(Pose, 'text_coordinate', self.text_callback, 10, callback_group=self.callback_group)
        # self.pose_subscriber = self.create_subscription(Pose, 'target_pose_in_base', self.target_pose_callback, 10, callback_group=self.callback_group)

        self.main_service = self.create_service(ArmControl, 'arm_control', self.run_task_service, callback_group=self.callback_group)

        config_path = os.path.join(get_package_share_directory('python_moveit2'), 'config', 'task_config.yaml')
        with open(config_path, 'r') as f:
            self.task_config = yaml.safe_load(f)

        self.get_logger().info('ArmControlRouter initialized.')

    def text_callback(self, msg):
        if self.text_receiving:
            self.get_logger().info(f"Text coordinate received: position=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})")
            self.latest_text_coordinate = msg
        else:
            self.get_logger().warn("Text coordinate not accepted (flag off).")

    def target_pose_callback(self, msg):
        self.latest_detect_pose = msg
        

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
            self.get_logger().error(f"Task failed due to exception: {str(e)}", exc_info=True)
            response.success = False
            response.message = f"Exception: {str(e)}"

        self.get_logger().info(f"Task [{task}] finished: {response.message}")
        return response


    def run_task_sequence(self, steps, response):
        for i, step in enumerate(steps):
            self.get_logger().info(f"Step {i+1}: {step}")
            result = None

            if 'name_pose' in step:
                result = self.send_named_pose(step['name_pose'], response)
                if result and result.success:
                    self.get_logger().info(f"Named pose '{step['name_pose']}' completed.")
                elif result and hasattr(result, 'failure') and result.failure:
                    self.get_logger().error(f"Named pose '{step['name_pose']}' failed.")
                    continue

            elif 'gripper' in step:
                close = step['gripper'] == 'close'
                result = self.send_gripper(close, response)
                if result and result.success:
                    self.get_logger().info(f"Gripper {'closed' if close else 'opened'} successfully.")
                elif result and hasattr(result, 'failure') and result.failure:
                    self.get_logger().error(f"Gripper {'close' if close else 'open'} failed.")
                    continue

            elif 'detect_pose' in step:
                if step['detect_pose']:
                    result = self.send_auto_pose(response)
                    if result and result.success:
                        self.get_logger().info("Auto pose detection completed.")
                    elif result and hasattr(result, 'failure') and result.failure:
                        self.get_logger().error("Auto pose detection failed.")
                        continue

            elif 'cartersian' in step:
                if step['cartersian']:
                    result = self.send_cartesian(response)
                    if result and result.success:
                        self.get_logger().info("Cartesian path completed.")
                    elif result and hasattr(result, 'failure') and result.failure:
                        self.get_logger().error("Cartesian path failed.")
                        continue

            elif 'log' in step:
                self.get_logger().info(f"{step['log']}")
                continue

            elif 'wait' in step:
                time.sleep(float(step['wait']))
                self.get_logger().info(f"Waited {step['wait']} seconds.")
                continue

            else:
                self.get_logger().warn(f"Unknown step: {step}")
                continue

            if result is None or not result.success:
                step_type = list(step.keys())[0] if isinstance(step, dict) else "unknown"
                if step_type == "gripper":
                    self.get_logger().warn(f"Gripper failed at step {i+1}, but continuing as it's allowed.")
                    continue
                self.get_logger().error(f"Step failed: {result.message if result else 'No response'}")
                response.success = False
                response.message = f"Aborted at step {i+1}: {step}" 
                return response


        response.success = True
        response.message = "All steps completed successfully."
        print("Task sequence finished.")
        return response

    def send_named_pose(self, name, response):
        self.get_logger().info("Waiting for named_pose_service...")
        if not self.named_pose_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Named pose service not available!")
            response.success = False
            response.message = "named_pose_service not available"
            return response
        self.get_logger().info("Named pose service is ready")

        req = PoseRequest.Request()
        req.message = name
        future = self.named_pose_client.call_async(req)
        return self.wait_for_future(future, response, "named_pose")


    def send_auto_pose(self, response):
        self.get_logger().info("Waiting for auto_pose_service...")
        if not self.auto_pose_client.wait_for_service(timeout_sec=10.0): 
            self.get_logger().error("Auto pose service not available!")
            response.success = False
            response.message = "auto_pose_service not available"
            return response
        self.get_logger().info("Auto pose service is ready")

        req = DetectPose.Request()
        future = self.auto_pose_client.call_async(req)
        return self.wait_for_future(future, response, "auto_pose")



    def send_cartesian(self, response):
        if not hasattr(self, 'cartesian_client'):
            response.success = False
            response.message = "cartesian_service client not defined"
            return response
        if not self.cartesian_client.wait_for_service(timeout_sec=0.5):
            response.success = False
            response.message = "cartesian_service not available"
            return response
        req = DetectPose.Request()
        future = self.cartesian_client.call_async(req)
        return self.wait_for_future(future, response, "cartesian")

    def send_gripper(self, close, response):
        
        if not self.gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Gripper service not available!")
            response.success = False
            response.message = "gripper_control_service not available"
            return response
        self.get_logger().info("Gripper service is ready")

        req = GripperControl.Request()
        req.close = close
        future = self.gripper_client.call_async(req)
        return self.wait_for_future(future, response, "gripper")
    
    def wait_for_future(self, future, response, label="service"):
        self.get_logger().info(f"Waiting for future result from {label}...")

        # Loop until the future is done
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)

        # Handle result
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"{label} exception: {str(e)}")
            response.success = False
            response.message = f"{label} exception: {str(e)}"
            return response
    

    # def wait_for_future(self, future, response, label="service"):
    #     if not future.done():
    #         self.get_logger().error(f"{label} future not completed.")
    #         response.success = False
    #         response.message = f"{label} future not completed."
    #         return response
    #     try:
    #         return future.result()
    #     except Exception as e:
    #         self.get_logger().error(f"{label} exception: {str(e)}")
    #         response.success = False
    #         response.message = f"{label} exception: {str(e)}"
    #         return response

    # def wait_for_future(self, future, response, label="service"):
    #     timeout = 10.0
    #     interval = 0.01
    #     waited = 0.0
    #     while not future.done() and waited < timeout:
    #         rclpy.spin_once(self, timeout_sec=interval)
    #         waited += interval

    #     if future.done():
    #         return future.result()
    #     else:
    #         self.get_logger().error(f"{label} timeout or did not respond.")
    #         response.success = False
    #         response.message = f"{label} timeout or did not respond."
    #         return 
    

    # def wait_for_future(self, future, response, label="service"):
    #     future.add_done_callback(lambda f: self._handle_future_result(f, response, label))
    #     # 返回一個標記表示正在處理
    #     return None

    # def _handle_future_result(self, future, response, label):
    #     try:
    #         result = future.result()
    #         # 處理結果並更新 response
    #     except Exception as e:
    #         response.success = False
    #         response.message = f"{label} failed: {str(e)}"

def main():
    rclpy.init()
    node = ArmControlRouter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
