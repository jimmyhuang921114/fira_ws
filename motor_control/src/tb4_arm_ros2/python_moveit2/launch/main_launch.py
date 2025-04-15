from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 啟動 MainControl (Python 節點)
        Node(
            package='fira_main',
            executable='main_control',
            name='main_control',
            output='screen'
        ),

        # 啟動 ArmControlRouter (Python 節點)
        Node(
            package='arm_interface',
            executable='arm_control_router',
            name='arm_control_router',
            output='screen'
        ),

        # 啟動 MoveIt C++ 節點
        Node(
            package='small_arm_moveit_cpp',
            executable='auto_move_node',
            name='auto_move_node',
            output='screen'
        )
    ])
