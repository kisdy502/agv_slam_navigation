cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(control
  src/pure_pursuit.cpp
  # src/pid.cpp
)

# ======================
# path publisher
# ======================

add_executable(path_publisher src/path_publisher.cpp)

ament_target_dependencies(
  path_publisher
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ======================
# pure pursuit node
# ======================

add_executable(pure_pursuit_node
  src/pure_pursuit_node.cpp
)

target_link_libraries(pure_pursuit_node
  control
)

ament_target_dependencies(
  pure_pursuit_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ======================
# install targets
# ======================
install(TARGETS
  control
  path_publisher
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
