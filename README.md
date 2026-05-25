# 录制所有话题（注意磁盘空间）

ros2 bag record -a -o agv_problem

# 或者只录制关键话题（节省空间）

ros2 bag record -o agv_problem \
 /cmd_vel \
 /odom \
 /ekf_odom \
 /amcl_pose \
 /scan \
 /tf \
 /tf_static \
 /plan \
 /local_plan \
 /goal_pose

# 带压缩录制

ros2 bag record -o agv_problem --compression-mode file --compression-format zstd \
 /cmd_vel /odom /ekf_odom /amcl_pose /scan

# 录制并限制时间,例如60秒

ros2 bag record -o agv_problem -d 60 /cmd_vel /odom /ekf_odom /amcl_pose

## example

ros2 bag record -o agv_problem -d 45 \
 /cmd_vel \
 /odom \
 /ekf_odom \
 /amcl_pose \
 /scan \
 /tf \
 /tf_static \
 /plan \
 /local_plan \
 /goal_pose

    rqt_plot 分析数据包

## 播放数据包

ros2 bag play agv_problem

# 循环播放（建图失败时反复测试）

ros2 bag play my_bag --loop

# 指定播放速率（加速/减速回放）

ros2 bag play my_bag --rate 0.5 # 0.5倍速
ros2 bag play my_bag --rate 2.0 # 2倍速

# 只播放指定话题（过滤噪声）

ros2 bag play my_bag --topics /scan /odom /imu

# 延迟控制（避免数据瞬间涌出）

ros2 bag play my_bag --max-skip 10 --read-ahead-queue-size 100

ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z

# 将两者画在一起

ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /odom/twist/twist/linear/x

ros2 run rqt_plot rqt_plot "/cmd_vel/linear/x" "/odom/twist/twist/linear/x"

ros2 run rqt_plot rqt_plot "/odom/twist/twist/linear/x"

## 导航节点运行日志

```

```

通读 Nav2 导航框架核心组件源码（nav2_smac_planner、nav2_regulated_pure_pursuit_controller、nav2_mppi_controller、nav2_behavior_tree），深入理解 Hybrid A* 的 SE2 状态空间搜索、Dubin/Reeds-Shepp 运动基元生成、Analytic Expansion 解析连接以及 Regulated Pure Pursuit 的多层调速与碰撞保护机制。能够从**运动学约束（最小转弯半径、非完整性）**出发，进行规划器/控制器的对比选型（Navfn vs SmacHybrid、MPPI vs RPP），并基于源码审计排查配置中的"幻觉参数"（不存在于源码的无效配置项），确保参数与算法行为的精确匹配。
熟练应用 A*/Dijkstra 进行全局路径规划，掌握 DWA/TEB/MPPI 等局部轨迹规划与避障算法的原理与适用场景，能根据底盘类型（差速/阿克曼/麦克纳姆）进行算法选型与参数调优。
基于 Nav2 Behavior Tree 机制，理解 Recovery 行为（Backup、DriveOnHeading）的触发逻辑与参数传递链路，能够根据阿克曼底盘的运动学限制定制故障恢复策略。
