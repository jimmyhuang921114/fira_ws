import time  # Time library
import math as m
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from transforms3d.euler import euler2quat
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from irobot_create_msgs.msg import InterfaceButtons, AudioNoteVector, AudioNote
from rclpy.node import Node
import toml, json
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler
from map_if.srv import Command

target_points = toml.load('./config/target_points.toml')
navigator: BasicNavigator = None


def get_audio_note(tone: int, dur: int):
    '''
        tone: 1-7 = C-B \n
        dur : 0-3 = 1.5, 1, 1/2, 1/4
    '''
    FREQ = [523, 587, 659, 698, 783, 880, 987]
    DUR = [1.5, 1, 0.5, 0.25]
    DUR_BASE = 600
    note = AudioNote()
    note.frequency = FREQ[tone-1]
    note.max_runtime = Duration(nanoseconds=DUR_BASE * 1e6 * DUR[dur]).to_msg()
    return note


# READY_NOTE_LIST = [
#     get_audio_note(3, 1),
#     get_audio_note(2, 2),
#     get_audio_note(1, 1),
#     get_audio_note(2, 2),
#     get_audio_note(3, 2),
#     get_audio_note(4, 3),
#     get_audio_note(3, 2),
#     get_audio_note(2, 0),
# ]

READY_NOTE_LIST = [
    get_audio_note(7, 3),
    get_audio_note(7, 3),
]

STEP_NOTE_LIST = [
    get_audio_note(5, 2),
]

START_NOTE_LIST = [
    get_audio_note(1, 1),
    get_audio_note(2, 1),
    get_audio_note(3, 1),
]

END_NOTE_LIST = [
    get_audio_note(3, 1),
    get_audio_note(2, 1),
    get_audio_note(1, 1),
]


def beep(node):
    audio_msg = AudioNoteVector()
    audio_msg.notes = STEP_NOTE_LIST
    node.audio_pub.publish(audio_msg)


class GoalPoint:
    def __init__(self, position: list[float, float], angle: float):
        '''
            position: [x,y] in meters\n
            angle: degree
        '''
        self.position = position
        self.angle = angle


def goal(points: list[GoalPoint]):
    waypoints = []
    for point in points:
        print(f"target: (x,y): {point.position}, angle: {point.angle}")
        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(point.position[0])
        goal_pose.pose.position.y = float(point.position[1])
        goal_pose.pose.position.z = 0.0
        quat = euler2quat(0, 0, m.radians(point.angle))
        goal_pose.pose.orientation.x = quat[1]
        goal_pose.pose.orientation.y = quat[2]
        goal_pose.pose.orientation.z = quat[3]
        goal_pose.pose.orientation.w = quat[0]
        # print(f"raw: {goal_pose}")
        waypoints.append(goal_pose)
    # Go to the goal pose
    # navigator.goToPose(goal_pose)
    navigator.followWaypoints(waypoints)
    # Keep doing stuff as long as the robot is moving towards the goal
    while not navigator.isTaskComplete():
        pass
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


def parse_points_cfg(points: list[list], sub_cfg: dict) -> list[GoalPoint]:
    goal_points = []
    for point in points:
        params = []
        for param in point:
            param = param[0]
            if isinstance(param, str):
                if sub_cfg.get(param):
                    params.append(sub_cfg[param])
                elif target_points.get(param):
                    params.append(target_points[param])
            else:
                params.append(param)
        goal_points.append(GoalPoint([params[0], params[1]], params[2]))
    return goal_points


def goal_target(src: str, dst: str):
    src_cfg: dict = target_points[src]
    dst_cfg: dict = target_points[dst]
    src_points_cfg: dict = None
    dst_points_cfg: dict = None
    ## src
    src_key = f"exit_{dst}"
    if src_cfg.get(src_key) is not None:
        src_points_cfg = src_cfg[src_key]
        print(f'src key: {src_key}')
    else:
        src_points_cfg = src_cfg['exit_default']
        print(f'src key: default')
    ## dst
    dst_key = f"enter_{src}"
    if dst_cfg.get(dst_key) is not None:
        dst_points_cfg = dst_cfg[dst_key]
        print(f'dst key: {dst_key}')
    else:
        dst_points_cfg = dst_cfg['enter_default']
        print(f'dst key: default')
    ## combine
    src_points = parse_points_cfg(src_points_cfg, src_cfg)
    dst_points = parse_points_cfg(dst_points_cfg, dst_cfg)
    goal(src_points + dst_points)


class PlatformNode(Node):
    def __init__(self):
        super().__init__('PlatformNode')
        self.button_sub = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.button_callback,
            2
        )
        self.audio_pub = self.create_publisher(
            AudioNoteVector,
            '/cmd_audio',
            2
        )
        self.cmd_srv = self.create_service(Command, 'command', self.cmd_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("node init")
        ## Variable
        self.start = False

    def button_callback(self, msg: InterfaceButtons):
        btn1 = msg.button_1
        if btn1.is_pressed \
                and btn1.last_pressed_duration.sec == 0 \
                and btn1.last_pressed_duration.nanosec == 0:
            print("start")
            audio_msg = AudioNoteVector()
            audio_msg.notes = START_NOTE_LIST
            self.audio_pub.publish(audio_msg)
            self.start = True
            time.sleep(2)

    def cmd_callback(self, request: Command.Request, response: Command.Response):
        ## not in start mode
        if not self.start:
            response.result = -2
            return response
        ## start mode
        self.get_logger().info(request.cmd)
        cmd: dict = json.loads(request.cmd)
        ## (src,dst) cmd
        if cmd.get('src') is not None:
            self.get_logger().info("(src,dst) cmd")
            goal_target(cmd['src'], cmd['dst'])
            beep(self)
            response.result = 1
        ## (relative move) cmd
        elif cmd.get('move') is not None:
            self.get_logger().info("(relative move) cmd")
            response.result = 2
        ## unknown
        else:
            self.get_logger().info("unknown cmd")
            response.result = -1
        time.sleep(3)
        return response

    def get_current_pose(self) -> GoalPoint | None:
        pose_tf = None
        try:
            pose_tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()).transform
            # print(pose_tf)
        except:
            return None
        quaternion = [pose_tf.rotation.w, pose_tf.rotation.x, pose_tf.rotation.y, pose_tf.rotation.z]
        roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')
        return GoalPoint([pose_tf.translation.x, pose_tf.translation.y], yaw)


'''
Navigates a robot from an initial pose to a goal pose.
'''
def main():
    global navigator
    # Start the ROS 2 Python Client Library
    rclpy.init() 
    node = PlatformNode()
    node.create_rate(20)

    navigator = BasicNavigator()

    # Set the robot's initial pose if necessary
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.position.z = 0.0
    # initial_pose.pose.orientation.x = 0.0
    # initial_pose.pose.orientation.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # navigator.setInitialPose(initial_pose)

    ## Wait for navigation to fully activate. Use this line if autostart is set to true.
    print("wait active")
    # navigator.waitUntilNav2Active()
    time.sleep(1)
    print("active")
    
    ## Ready
    audio_msg = AudioNoteVector()
    audio_msg.notes = READY_NOTE_LIST
    node.audio_pub.publish(audio_msg)
    
    node.start = True
    goal_target('init', 'init')

    while True:
        rclpy.spin_once(node)

    exit(0)


if __name__ == '__main__':
    main()