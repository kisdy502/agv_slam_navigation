# ekf_tests

扩展卡尔曼滤波（EKF）定位测试包，用于融合里程计和传感器观测数据。

## 创建包

```bash
cd /home/kisdy/projects/agv_localization_ws/src
ros2 pkg create ekf_tests --build-type ament_cmake --dependencies rclcpp eigen
```

## 依赖

| 依赖 | 说明 |
|------|------|
| rclcpp | ROS2 C++ 客户端库 |
| Eigen | 线性代数库（矩阵运算） |
| nav_msgs | 里程计消息 |
| geometry_msgs | 位姿消息 |
| sensor_msgs | 传感器消息 |
| tf2 | 坐标变换 |

## 编译

```bash
cd /home/kisdy/projects/agv_localization_ws
colcon build --packages-select ekf_tests
source install/setup.bash
```

## 运行

```bash
# 启动 EKF 节点
ros2 run ekf_tests ekf_localization_node --ros-args -p pose_topic:=/amcl_pose
```

## 代码结构

```
ekf_tests/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── ekf_tests/
│       └── ekf_localization.hpp    # EKF 类声明
└── src/
    ├── ekf_localization.cpp         # EKF 算法实现
    └── ekf_localization_node.cpp   # ROS2 节点
```

## 算法说明

### EKF 两步法

```
              预测阶段                    更新阶段
              
里程计 ──→ predict() ──→ 状态预测 ──→ update() ──→ 状态更新
                │                            │
                ↓                            ↓
           协方差增大                    协方差减小
```

### 状态向量

```
x_ = [x, y, θ]ᵀ  // 位置 + 朝向
```

### 预测阶段 (predict)

- **输入**: 速度 `v`, 角速度 `ω`, 时间间隔 `dt`
- **状态更新**:
  ```
  x' = x + v·cos(θ)·dt
  y' = y + v·sin(θ)·dt
  θ' = θ + ω·dt
  ```
- **协方差更新**: `P' = F·P·Fᵀ + Q`

### 更新阶段 (update_landmark)

- **输入**: 观测位置 `(z_x, z_y)`
- **卡尔曼增益**: `K = P'·Hᵀ·(H·P'·Hᵀ + R)⁻¹`
- **状态更新**: `x = x' + K·(z - H·x')`
- **协方差更新**: `P = (I - K·H)·P'`

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/odom` | nav_msgs/Odometry | 订阅 | 里程计数据 |
| `/gps` | geometry_msgs/PoseStamped | 订阅 | GPS/定位观测 |
| `/ekf_odom` | nav_msgs/Odometry | 发布 | EKF 融合结果 |

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `q_v_` | 0.1 | 速度过程噪声 |
| `q_omega_` | 0.05 | 角速度过程噪声 |
| `R` | 0.1·I | 观测噪声协方差 |

## 扩展

室内机器人可将 `/gps` 替换为 ICP/激光配准的输出，实现室内定位融合。
