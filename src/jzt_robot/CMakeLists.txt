cmake_minimum_required(VERSION 3.8)
project(jzt_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)      # 也补上
find_package(rclcpp_components REQUIRED)  # 也补上
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED) # Joy message
find_package(geometry_msgs REQUIRED) # Twist message
find_package(nav_msgs REQUIRED) # Odometry message
find_package(tf2 REQUIRED)              # 新增！
find_package(tf2_ros REQUIRED) # Transform broadcaster
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)

find_package(jzt_robot_msgs REQUIRED) # 你的自定义消息包
find_package(pluginlib REQUIRED)



include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(src/utils)

# Gamepad Teleop Node
add_executable(gamepad_teleop_node
  src/gamepad_teleop_node.cpp
)

ament_target_dependencies(gamepad_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Odom Relay Node
add_executable(odom_relay_node
  src/odom_relay_node.cpp
)

ament_target_dependencies(odom_relay_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)
# cmd_vel_relay_node
add_executable(cmd_vel_relay_node
  src/cmd_vel_relay_node.cpp
)

ament_target_dependencies(cmd_vel_relay_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# cmd_vel_mux Node
add_executable(cmd_vel_mux_node
  src/cmd_vel_mux.cpp
)

ament_target_dependencies(cmd_vel_mux_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# ackermann_verifier_node
add_executable(ackermann_verifier_node
  src/ackermann_verifier_node.cpp
)

ament_target_dependencies(ackermann_verifier_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
)

# ackermann_calib_trajectory_node
add_executable(ackermann_calib_trajectory_node
  src/ackermann_calib_trajectory_node.cpp
)

ament_target_dependencies(ackermann_calib_trajectory_node
  rclcpp
  geometry_msgs
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs          
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_util
  jzt_robot_msgs
  pluginlib
)


# ========== DockingController 组件 ==========
add_library(docking_controller SHARED
  src/docking_controller.cpp
)
target_include_directories(docking_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(docking_controller ${dependencies})
rclcpp_components_register_nodes(docking_controller "jzt_robot::DockingController")

# ========== BT DockingAction 插件 ==========
add_library(bt_docking_action SHARED
  src/bt_docking_action.cpp
)
target_include_directories(bt_docking_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_docking_action ${dependencies})

target_compile_definitions(bt_docking_action PRIVATE BT_PLUGIN_EXPORT)

# 关键：设置符号默认可见，否则 BT_RegisterNodesFromPlugin 被隐藏
set_target_properties(bt_docking_action PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN NO
)

# ========== BT ComputeStagingPose 插件 ==========
add_library(bt_compute_staging_pose SHARED
  src/bt_compute_staging_pose.cpp
)
# BUGFIX: 原来写的是 ... 占位符，导致 bt_compute_staging_pose 编译时找不到 include 路径
target_include_directories(bt_compute_staging_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_compute_staging_pose ${dependencies})
target_compile_definitions(bt_compute_staging_pose PRIVATE BT_PLUGIN_EXPORT)
set_target_properties(bt_compute_staging_pose PROPERTIES CXX_VISIBILITY_PRESET default VISIBILITY_INLINES_HIDDEN NO)

# ========== BT SetNav2Param 插件 ==========
add_library(bt_set_nav2_param SHARED
  src/bt_set_nav2_param.cpp
)
target_include_directories(bt_set_nav2_param PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_set_nav2_param ${dependencies})
target_compile_definitions(bt_set_nav2_param PRIVATE BT_PLUGIN_EXPORT)
set_target_properties(bt_set_nav2_param PROPERTIES CXX_VISIBILITY_PRESET default VISIBILITY_INLINES_HIDDEN NO)

# 导出 BT 插件
pluginlib_export_plugin_description_file(behaviortree_cpp_v3 plugins.xml)


# Install executables
install(TARGETS
  gamepad_teleop_node
  odom_relay_node
  cmd_vel_relay_node
  ackermann_verifier_node
  ackermann_calib_trajectory_node
  cmd_vel_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

# ========== 安装 ==========
install(TARGETS docking_controller bt_docking_action bt_compute_staging_pose bt_set_nav2_param
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY
  param/
  DESTINATION share/${PROJECT_NAME}/param
)
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY
  world/
  DESTINATION share/${PROJECT_NAME}/world
)

install(DIRECTORY
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY
  meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
