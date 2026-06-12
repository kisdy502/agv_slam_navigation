# triple_steer_robot

三舵轮机器人仿真包，支持 Gazebo 仿真和 3D LiDAR 传感器仿真。

## 编译

```bash
cd /home/kisdy/projects/agv_localization_ws
colcon build --packages-select triple_steer_robot --symlink-install
source install/setup.bash
```

## 启动顺序

### 1. 启动 Gazebo 三舵轮仿真

```bash
conda deactivate
source install/setup.bash
ros2 launch triple_steer_robot gazebo_sim.launch.py
```

#### 指定世界文件

```bash
ros2 launch triple_steer_robot gazebo_sim.launch.py world_name:=jzt_factory_sz.world
```

> **重要**: Gazebo 启动后保持运行，不要关闭。
> 当前机器人仅展示静态模型，后续将支持 `/cmd_vel` 运动控制。

### 2. 查看机器人模型与传感器（RViz）

```bash
conda deactivate
source install/setup.bash
ros2 launch triple_steer_robot gazebo_sim.launch.py
# 新开终端
rviz2 -d $(ros2 pkg prefix triple_steer_robot)/share/triple_steer_robot/rviz/robot.rviz
```

### 3. 查看 3D 雷达点云

```bash
# 实时查看点云数据
ros2 topic echo /lidar_3d/points --once

# 查看点云发布频率
ros2 topic hz /lidar_3d/points
```

### 4. 查看 IMU 数据

```bash
ros2 topic echo /imu/data --once | grep frame_id
ros2 topic hz /imu/data
```

---

## 常用调试命令

### 查看话题

```bash
# 查看所有 topic
ros2 topic list

# 查看 topic 类型
ros2 topic type /lidar_3d/points

# 查看 topic 结构
ros2 interface show sensor_msgs/msg/PointCloud2

# 实时打印 topic 数据
ros2 topic echo /lidar_3d/points

# 查看 topic 发布频率
ros2 topic hz /lidar_3d/points

# 查看 topic 带宽
ros2 topic bw /lidar_3d/points
```

### 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

### 查看节点参数

```bash
ros2 param list /robot_state_publisher
ros2 param get /robot_state_publisher robot_description
```

### URDF 转换与检查

```bash
# xacro 转 urdf
xacro src/triple_steer_robot/urdf/triple_steer_robot.urdf.xacro > /tmp/triple_steer.urdf

# 检查 urdf 有效性
check_urdf /tmp/triple_steer.urdf
```

---

## Launch 文件说明

| 文件 | 功能 |
| --- | --- |
| `gazebo_sim.launch.py` | Gazebo 仿真环境 + 机器人生成 + 控制器加载 |

---

## 文件结构

```
triple_steer_robot/
├── launch/
│   └── gazebo_sim.launch.py       # Gazebo 仿真启动
├── config/
│   ├── gazebo_params.yaml         # Gazebo 服务器参数
│   └── ros2_control.yaml          # ros2_control 控制器配置
├── param/
│   └── nav2_params.yaml           # Nav2 参数占位
├── rviz/
│   └── robot.rviz                 # RViz 配置文件
├── urdf/
│   ├── triple_steer_robot.urdf.xacro   # 顶层 URDF
│   ├── robot_base.xacro           # 三舵轮底盘模型
│   ├── sensors.xacro              # 3D LiDAR + IMU
│   └── ros2_control.xacro         # ros2_control 插件配置
├── world/
│   └── empty.world                # 空世界文件
├── maps/                          # 地图文件目录（预留）
├── meshes/                        # 网格模型目录（预留）
├── behavior_trees/                # 行为树目录（预留）
├── src/                           # 源码目录（预留）
├── include/                       # 头文件目录（预留）
├── package.xml
├── CMakeLists.txt
└── README.md
```

---

## TODO / 后续计划

1. **三舵轮逆运动学节点**：实现 `cmd_vel` 到三个轮子的转速和转角解算
2. **自定义 ros2_control 控制器**：或基于 `forward_command_controller` 配合 IK 节点
3. **3D LiDAR 插件增强**：可替换为 `velodyne_gazebo_plugins` 获得更逼真的仿真
4. **建图与导航**：集成 Cartographer 3D 建图 + Nav2 导航栈
5. **外观模型**：替换基础几何体为实际 STL/DAE 网格模型
