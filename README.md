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
Start navigation
[rviz2-12] [INFO] [1777280071.080551460] [rviz_navigation_dialog_action_client]: NavigateToPose will be called using the BT Navigator's default behavior tree.
[bt_navigator-7] [INFO] [1777280071.081641464] [bt_navigator]: Begin navigating from current location (-14.37, -7.68) to (-9.25, -3.72)
[controller_server-3] [INFO] [1777280071.283063745] [controller_server]: Received a goal, begin computing control effort.
[controller_server-3] [WARN] [1777280071.283151970] [controller_server]: No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded general_goal_checker . This warning will appear once.
[controller_server-3] [INFO] [1777280071.288668378] [controller_server]: Optimizer reset
[controller_server-3] [INFO] [1777280072.583368369] [controller_server]: Passing new path to controller.
[gamepad_teleop_node-13] [INFO] [1777280073.449274046] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.500208643] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.551063480] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.651736680] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.701470084] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.751420294] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.861547479] [gamepad_teleop_node]: 发送了停止速度控制！
[controller_server-3] [INFO] [1777280073.883348946] [controller_server]: Passing new path to controller.
[gamepad_teleop_node-13] [INFO] [1777280073.911079455] [gamepad_teleop_node]: 发送了停止速度控制！
[gamepad_teleop_node-13] [INFO] [1777280073.961198411] [gamepad_teleop_node]: 发送了停止速度控制！
[controller_server-3] [INFO] [1777280075.183351754] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280076.483348482] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280077.783349056] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280079.083351113] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280080.383353742] [controller_server]: Passing new path to controller.
[controller_server-3] [ERROR] [1777280081.283444247] [controller_server]: Failed to make progress
[controller_server-3] [WARN] [1777280081.283597595] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[controller_server-3] [INFO] [1777280081.682306922] [controller_server]: Received a goal, begin computing control effort.
[controller_server-3] [INFO] [1777280082.982457738] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280084.282486196] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280085.632536280] [controller_server]: Passing new path to controller.
[rviz2-12] Start navigation
[rviz2-12] [INFO] [1777280086.040063960] [rviz_navigation_dialog_action_client]: NavigateToPose will be called using the BT Navigator's default behavior tree.
[bt_navigator-7] [INFO] [1777280086.081851561] [bt_navigator]: Received goal preemption request
[bt_navigator-7] [INFO] [1777280086.082154429] [bt_navigator]: Begin navigating from current location (-14.13, -8.05) to (-2.36, -1.82)
[controller_server-3] [INFO] [1777280086.932460121] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280088.232458622] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280089.482455592] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280090.782462016] [controller_server]: Passing new path to controller.
[controller_server-3] [ERROR] [1777280091.682504155] [controller_server]: Failed to make progress
[controller_server-3] [WARN] [1777280091.682622588] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[controller_server-3] [INFO] [1777280092.082334339] [controller_server]: Received a goal, begin computing control effort.
[controller_server-3] [INFO] [1777280093.382498971] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280094.682496368] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280095.982502621] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280097.282498485] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280098.582498757] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280099.882500952] [controller_server]: Passing new path to controller.
[controller_server-3] [INFO] [1777280101.182499750] [controller_server]: Passing new path to controller.
[controller_server-3] [ERROR] [1777280102.082557161] [controller_server]: Failed to make progress
[controller_server-3] [WARN] [1777280102.082661116] [controller_server]: [follow_path] [ActionServer] Aborting handle.
```