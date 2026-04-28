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
ros2 bag play my_bag --rate 0.5    # 0.5倍速
ros2 bag play my_bag --rate 2.0    # 2倍速

# 只播放指定话题（过滤噪声）
ros2 bag play my_bag --topics /scan /odom /imu

# 延迟控制（避免数据瞬间涌出）
ros2 bag play my_bag --max-skip 10 --read-ahead-queue-size 100


ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z
# 将两者画在一起
ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /odom/twist/twist/linear/x

ros2 run rqt_plot rqt_plot "/cmd_vel/linear/x" "/odom/twist/twist/linear/x"

ros2 run rqt_plot rqt_plot  "/odom/twist/twist/linear/x"




## 导航节点运行日志
```
```