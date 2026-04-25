## A*算法网格处理

```
colcon build --packages-select agv_global_planner
source install/setup.bash
```

# 启动规划器（需要提供地图和定位）
```
ros2 launch agv_global_planner astar_planner.launch.py
```