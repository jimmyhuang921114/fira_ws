 import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Build moveit config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="small_arm", package_name="small_arm_moveit_config"
        )
        .robot_description(file_path="config/small_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/small_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("small_arm_moveit_cpp"),
                "config/moveit_cpp.yaml"
            )
        )
        .to_moveit_configs()
    )

    auto_pose_node = Node(
        name="moveit_auto_pose",
        package="small_arm_moveit_cpp",
        executable="auto_pose_goal",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    ee_pose_node = Node(
        name="moveit_ee_pose",
        package="small_arm_moveit_cpp",
        executable="ee_pose_in_base",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    gripper_open_node = Node(
       name="moveit_gripper_open",
       package="small_arm_moveit_cpp",
       executable="gripper_open",
       output="screen",
       parameters=[moveit_config.to_dict()],
    ) 

    return LaunchDescription([
        auto_pose_node,
        ee_pose_node,
        gripper_open_node,
    ])
