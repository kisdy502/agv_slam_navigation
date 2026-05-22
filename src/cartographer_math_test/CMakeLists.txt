cmake_minimum_required(VERSION 3.8)
project(cartographer_math_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartographer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED) # 后续使用

# Test node for Rigid3d transform
add_executable(test_rigid_transform
  src/test_rigid_transform.cpp
)
target_link_libraries(test_rigid_transform
  cartographer
  ${rclcpp_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(test_rigid_transform
  rclcpp
  cartographer
)

# Future: test node for Ceres optimization (placeholder)
add_executable(test_ceres_optimization
  src/test_ceres_optimization.cpp
)
target_link_libraries(test_ceres_optimization
  cartographer
  ${rclcpp_LIBRARIES}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(test_ceres_optimization
  rclcpp
  cartographer
)

# Test node for lidar polar-to-cartesian transform
add_executable(test_lidar_polar_to_cartesian
  src/test_lidar_polar_to_cartesian.cpp
)
target_link_libraries(test_lidar_polar_to_cartesian
  ${rclcpp_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(test_lidar_polar_to_cartesian
  rclcpp
)

# Test node for 2D affine transform
add_executable(test_affine_transform
  src/test_affine_transform.cpp
)
target_link_libraries(test_affine_transform
  cartographer
  ${rclcpp_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(test_affine_transform
  rclcpp
)

# Install executables
install(TARGETS
  test_rigid_transform
  test_ceres_optimization
  test_lidar_polar_to_cartesian
  test_affine_transform
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
