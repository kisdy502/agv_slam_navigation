#!/usr/bin/env python3
"""
阿克曼底盘 Gazebo 仿真启动文件（使用 xacro + tempfile 方式）
"""

import os
import xacro
import tempfile
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler,
    TimerAction, LogInfo
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "jzt_robot"

    robot_name_in_model = "jzAckermannRobot01"
    xacro_name = "jz_ackermann_robot.urdf.xacro"          # 顶层 xacro 文件
    gazebo_params_file_name = "gazebo_params.yaml"

    pkg_share = FindPackageShare(package_name).find(package_name)
    xacro_model_path = os.path.join(pkg_share, f"urdf/ackermann/{xacro_name}")
    gazebo_params_path = os.path.join(pkg_share, f"config/{gazebo_params_file_name}")

    # ---------- 参数声明 ----------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用 /clock 话题作为时间源'
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Launch Gazebo GUI'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # 如果有需要可添加其他参数，例如控制话题等
    declare_cmd_topic = DeclareLaunchArgument(
        'cmd_topic', default_value='/ackermann_steering_controller/reference_unstamped',
        description='阿克曼速度控制话题'
    )
    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='jzt_work_room.world',
        description='Gazebo 世界文件名（位于 world/ 目录下）'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    yaw_angle = LaunchConfiguration('yaw')
    world_name = LaunchConfiguration('world_name')

    gazebo_world_path = PathJoinSubstitution([pkg_share, 'world', world_name])

    # ---------- xacro 处理 ----------
    # 注意：use_gazebo 使用 xacro arg 默认值 1，无需 mappings 覆盖
    doc = xacro.process_file(xacro_model_path)
    robot_desc = doc.toprettyxml(indent='  ')

    # 写进临时文件
    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp_urdf.write(robot_desc)
    tmp_urdf.close()
    urdf_file_path = tmp_urdf.name

    # ========== 1. 启动 Gazebo ==========
    start_gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path,
            '--ros-args', '--params-file', gazebo_params_path,
        ],
        output='screen',
    )

    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    # ========== 2. Robot State Publisher ==========
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_desc,
                'use_sim_time': use_sim_time
            },
        ],
    )

    # ========== 3. Spawn Robot ==========
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name_in_model,
            "-file", urdf_file_path,
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
            "-Y", yaw_angle,
            "-robot_namespace", "/",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ========== 4. 控制器加载 ==========
    # 关节状态广播器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 阿克曼转向控制器（将控制器命名空间下的 odometry 话题重映射到 /odom）
    load_ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "-c", "/controller_manager",
            # "--ros-args",
            # "-r", "/ackermann_steering_controller/odometry:=/odom",
            # "-r", "~/tf_odometry:=/tf_odometry",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ========== 5. 事件链 ==========
    spawn_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gzserver,
            on_start=[
                LogInfo(msg="✅ Gazebo server 已启动 → 3秒后开始Spawn Robot"),
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg="🚀 正在Spawn机器人..."),
                        spawn_entity,
                    ]
                )
            ]
        )
    )

    controllers_after_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                LogInfo(msg="✅ Robot spawned → 3秒后加载控制器"),
                TimerAction(
                    period=3.0,
                    actions=[
                        load_joint_state_broadcaster,
                        TimerAction(
                            period=1.0,
                            actions=[load_ackermann_controller],
                        ),
                    ],
                ),
            ]
        )
    )

    # 里程计中继：控制器发布 /ackermann_steering_controller/odometry，转发到 /odom
    odom_relay_node = Node(
        package="jzt_robot",
        executable="odom_relay_node",
        output="screen",
        parameters=[
            {'input_topic': '/ackermann_steering_controller/odometry'},
            {'output_topic': '/odom'},
            {'publish_tf': False},  # TF 由控制器 enable_odom_tf 发布
            {"use_sim_time": use_sim_time}
        ],
    )
    
    cmd_vel_relay_node = Node(
        package="jzt_robot",
        executable="cmd_vel_relay_node",
        output="screen",
        parameters=[
            {'output_topic': '/ackermann_steering_controller/reference_unstamped'},
            {"use_sim_time": use_sim_time}
        ],
    )

    # ---------- LaunchDescription 组装 ----------
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)
    ld.add_action(declare_cmd_topic)   # 如果你后续需要用到
    ld.add_action(declare_world_name)

    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(spawn_after_gazebo)
    ld.add_action(controllers_after_spawn)
    ld.add_action(odom_relay_node)
    ld.add_action(cmd_vel_relay_node)

    return ld