# jzt_robot

AGV 机器人仿真包，支持 Gazebo 建图和导航。

## 启动顺序

### 1. 启动 Gazebo 差速仿真（始终运行）

```bash
conda deactivate
source install/setup.bash
ros2 launch jzt_robot gazebo_diff.launch.py
```

### 1. 启动 Gazebo 阿克曼仿真（始终运行）差速，阿克曼二选一

```bash
conda deactivate
source install/setup.bash
ros2 launch jzt_robot gazebo_ackermann.launch.py
```

> **重要**: Gazebo 启动后保持运行，不要关闭。无论建图还是导航都基于此仿真环境。
> **重要**: 阿克曼的底盘，移动控制话题和差速不一样，用的是
> **重要**: ros2_control (ackermann_steering_controller)  /ackermann_steering_controller/reference_unstamped
> **重要**: ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.6}, angular: {z: 0.4}}' --rate 5


---

### 2. 建图模式

```bash
# 在新的终端中启动建图 + RViz
conda deactivate
source install/setup.bash
ros2 launch jzt_robot slam.launch.py

# 键盘控制机器人移动（第三个终端）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**建图完成后保存地图:**
```bash
ros2 service call /write_state \
    cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/kisdy/maps/cartographer_map.pbstream'}"
```

> 建图完成后，关闭建图终端（Ctrl+C），再启动导航模式。

---

### 3. 导航模式

<!-- cspell: ignore pbstream kisdy -->

```bash
# 先确保 gazebo 正在运行，然后启动导航 + RViz
conda deactivate
source install/setup.bash
ros2 launch jzt_robot navigation.launch.py \
    pbstream_file:=/home/kisdy/maps/cartographer_map.pbstream
```

---

## Launch 文件说明

| 文件 | 功能 |
|------|------|
| `gazebo_diff.launch.py` | Gazebo 仿真环境（需始终运行） |
| `slam.launch.py` | Cartographer 建图 + RViz 可视化 |
| `navigation.launch.py` | Nav2 导航 + Cartographer 定位 + RViz |

## 文件结构

```
jzt_robot/
├── launch/
│   ├── gazebo_diff.launch.py              # Gazebo 仿真
│   ├── slam.launch.py                # 建图模式
│   ├── navigation.launch.py          # 导航模式
├── config/                          # 机器人配置
├── param/                           # Nav2 参数
├── rviz/                            # RViz 配置文件
├── urdf/                            # 机器人模型
└── world/                           # Gazebo 世界文件
```

## 编译

```bash
colcon build --packages-select jzt_robot
```


## jzt_robot 问题大全 vs 解决步骤
 
 
 1.建图时候，每次重启建图，机器人在地图位置发生偏移了，gazebo和Cartographer重复发布odom到tf坐标变化了
 2.建图雷达不贴合地图边缘，精度不够，修改差速urdf，将分辨率提高到720，起始结束角度和插件保持一致，降低雷达的噪声
 3.启动nav2 导航launch报错 原因，我现在用Cartographer建图，Cartographer导航，不能用bring_up.launch.py会默认启动amcl
 需改成 navigation_launch.py
 4.nav2启动rviz看不到机器人位置，ros2 run tf2_tools view_frames，发现没有map到odom的坐标变化，原因gazebo和Cartographer分别发布了map-odom，base_footprint-odom坐标变化，导致冲突


 ## 检查imu 的frame id是否正确
```
 ros2 topic echo /imu --once | grep frame_id
  frame_id: imu_link
```

## 修复了问题
```
1启动导航节点，地图不现实，日志提示odom到base_footprint坐标不存在，底盘urdf配置问题，imu的frame id需要设置成imu_link
     localization_2d.lua参数问题，参考官方backup_2d.lua配置，补齐参数
1导航节点启动时候，rviz中雷达轮廓和地图边缘不贴合，
  差速底盘urdf配置有问题，lidar的frame id设置成laser_scan
  localization_2d.lua参数问题，参考官方backpack_2d_localization.lua，补齐参数
```

