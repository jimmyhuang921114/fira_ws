cmake_minimum_required(VERSION 3.8)
project(small_arm_moveit_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(python_moveit_interface REQUIRED)  # ✅ 這行是關鍵

add_executable(ee_pose_in_base src/ee_pose_in_base.cpp)
add_executable(auto_pose_goal src/auto_pose_goal.cpp)
add_executable(named_goal src/named_goal.cpp)
add_executable(gripper_open src/gripper_open.cpp)
add_executable(manual_pose_topic src/manual_pose_topic.cpp)

# 加入必要依賴
ament_target_dependencies(ee_pose_in_base
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  python_moveit_interface
)

ament_target_dependencies(auto_pose_goal
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  python_moveit_interface
)

ament_target_dependencies(named_goal
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  python_moveit_interface
)

ament_target_dependencies(gripper_open
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  python_moveit_interface
)

ament_target_dependencies(manual_pose_topic
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  python_moveit_interface
)

install(TARGETS
  auto_pose_goal
  named_goal
  gripper_open
  ee_pose_in_base
  manual_pose_topic
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

