import rclpy
import json
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node as LaunchNode
from std_msgs.msg import String
import subprocess, signal, os


class NodeManager(Node):
    def __init__(self):
        super().__init__('node_manager')
        self.process = None
        self.cmd_sub = self.create_subscription(
            String,
            '/command',
            self.command_callback,
            2
        )

    def command_callback(self, msg: String):
        command = json.loads(msg.data)
        print(command)
        if command['enable']:
            self.process = subprocess.Popen(f"ros2 run {command['package']} {command['node']}", shell=True, preexec_fn=os.setsid)
        elif self.process is not None:
            print('term')
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

    def launch_other_node(self):
        self.get_logger().info('Launching another node...')
        self.process = subprocess.Popen(['ros2', 'run', 'your_package', 'your_node'])

    def stop_other_node(self):
        if self.process:
            self.get_logger().info('Stopping the other node...')
            self.process.terminate()
            self.process.wait()


def main(args=None):
    rclpy.init(args=args)
    node_manager = NodeManager()

    try:
        # node_manager.launch_other_node()
        rclpy.spin(node_manager)
    except KeyboardInterrupt:
        pass
    finally:
        # node_manager.stop_other_node()
        node_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
