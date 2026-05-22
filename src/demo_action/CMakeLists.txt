cmake_minimum_required(VERSION 3.8)
project(demo_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成 Action 接口（.action → .hpp）
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountToN.action"
  DEPENDENCIES std_msgs
)

# 创建可执行文件
add_executable(count_server src/count_server.cpp)
ament_target_dependencies(count_server rclcpp rclcpp_action std_msgs)

add_executable(count_client src/count_client.cpp)
ament_target_dependencies(count_client rclcpp rclcpp_action std_msgs)

add_executable(count_server_lifecycle src/count_server_lifecycle.cpp)
ament_target_dependencies(count_server_lifecycle rclcpp rclcpp_action rclcpp_lifecycle std_msgs)

# 链接自动生成的接口库
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(count_server ${cpp_typesupport_target})
target_link_libraries(count_client ${cpp_typesupport_target})
target_link_libraries(count_server_lifecycle ${cpp_typesupport_target})

# 关键！显式声明：server/client 必须等接口生成完再编译
add_dependencies(count_server ${PROJECT_NAME})
add_dependencies(count_client ${PROJECT_NAME})
add_dependencies(count_server_lifecycle ${PROJECT_NAME})

install(TARGETS
  count_server
  count_client
  count_server_lifecycle
  DESTINATION lib/${PROJECT_NAME})


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
