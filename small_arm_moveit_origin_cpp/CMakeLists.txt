cmake_minimum_required(VERSION 3.8)
project(small_arm_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # Add this dependency

add_executable(pose_goal src/pose_goal.cpp)
ament_target_dependencies(pose_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(joint_goal src/joint_goal.cpp)
ament_target_dependencies(joint_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(cartesian_path src/cartesian_path.cpp)
ament_target_dependencies(cartesian_path
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(named_goal src/named_goal.cpp)
ament_target_dependencies(named_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(gripper_open src/gripper_open.cpp)
ament_target_dependencies(gripper_open
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(gripper_joint_value src/gripper_joint_value.cpp)
ament_target_dependencies(gripper_joint_value
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(pliz_move src/pliz_move.cpp)
ament_target_dependencies(pliz_move
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(tool_move src/tool_coordinate_move.cpp)
ament_target_dependencies(tool_move
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(ee_pose_in_base src/ee_pose_in_base.cpp)
ament_target_dependencies(ee_pose_in_base
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(eye_hand_pose src/eye_hand_pose.cpp)
ament_target_dependencies(eye_hand_pose
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS eye_hand_pose DESTINATION lib/${PROJECT_NAME})
install(TARGETS ee_pose_in_base DESTINATION lib/${PROJECT_NAME})
install(TARGETS pose_goal DESTINATION lib/${PROJECT_NAME})  
install(TARGETS joint_goal DESTINATION lib/${PROJECT_NAME})
install(TARGETS cartesian_path DESTINATION lib/${PROJECT_NAME})
install(TARGETS named_goal DESTINATION lib/${PROJECT_NAME})
install(TARGETS gripper_open DESTINATION lib/${PROJECT_NAME})
install(TARGETS gripper_joint_value DESTINATION lib/${PROJECT_NAME})
install(TARGETS pliz_move DESTINATION lib/${PROJECT_NAME})
install(TARGETS tool_move DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
