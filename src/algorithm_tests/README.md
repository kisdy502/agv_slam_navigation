# algorithm_tests

算法测试包，用于测试各种定位和配准算法。

## 创建包

```bash
cd /home/kisdy/projects/agv_localization_ws/src
ros2 pkg create algorithm_tests --build-type ament_cmake --dependencies rclcpp eigen
```

## 编译

```bash
cd /home/kisdy/projects/agv_localization_ws
colcon build --packages-select algorithm_tests --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

## 运行测试

```bash
# ICP 测试
source install/setup.bash
ros2 run algorithm_tests algorithm_test_node --ros-args -p type:=ipc

# Matrix Demo 测试（新增）
ros2 run algorithm_tests algorithm_test_node --ros-args -p type:=matrix

```

## 添加新算法

1. 在 `include/` 下添加对应的 hpp 头文件
2. 在 `src/` 下添加 cpp 实现文件和 `test()` 方法
3. 在 `src/main.cpp` 中添加对应的类型判断分支