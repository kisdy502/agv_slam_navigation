## A*算法网格处理

```
colcon build --packages-select agv_global_planner
source install/setup.bash
```

# 启动规划器（需要提供地图和定位）
```
ros2 launch agv_global_planner astar_planner.launch.py
```


# 常用算法
```
高斯分布
卡尔曼滤波
扩展卡尔曼滤波
粒子滤波
最大似然
最小二乘
贝叶斯估计
SFM、BA和SLAM比较
LOAM
ORB-SLAM2
RTAB-MAP
```