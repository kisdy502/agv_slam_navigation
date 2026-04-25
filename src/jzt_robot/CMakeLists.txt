cmake_minimum_required(VERSION 3.8)
project(jzt_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)  # Joy message
find_package(geometry_msgs REQUIRED) # Twist message

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Gamepad Teleop Node
add_executable(gamepad_teleop_node
  src/gamepad_teleop_node.cpp
)

ament_target_dependencies(gamepad_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  gamepad_teleop_node
  DESTINATION lib/${PROJECT_NAME}
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
