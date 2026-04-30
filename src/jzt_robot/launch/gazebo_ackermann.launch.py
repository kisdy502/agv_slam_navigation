#!/usr/bin/env python3
"""
阿克曼底盘 Gazebo 仿真启动文件（gazebo_ros2_control 版本）
"""

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler,
    TimerAction, LogInfo
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = "jzAckermannRobot01"
    package_name = "jzt_robot"
    world_name = "jzt_factory_sz.world"
    urdf_name = "jzt_ackermann.urdf"
    gazebo_params_file_name = "gazebo_params.yaml"

    pkg_share = FindPackageShare(package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    gazebo_world_path = os.path.join(pkg_share, f"world/{world_name}")
    gazebo_params_path = os.path.join(pkg_share, f"config/{gazebo_params_file_name}")

    # 声明参数
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Launch Gazebo GUI'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    gui = LaunchConfiguration('gui')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    yaw_angle = LaunchConfiguration('yaw')

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

    # ========== 2. Robot State Publisher（必须在 spawn 前启动）==========
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": open(urdf_model_path, 'r').read()},
            {"use_sim_time": True}
        ],
    )

    # ========== 3. Spawn Robot（关键修复：改用 -topic）==========
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name_in_model,
            "-topic", "/robot_description",
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
            "-Y", yaw_angle,
            "-robot_namespace", "/",  # 命名空间
        ],
        output="screen",
    )

    # ========== 4. 控制器加载 ==========
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    load_ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    
    odom_relay_node = Node(
        package="jzt_robot",
        executable="odom_relay_node",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )
    
    cmd_vel_relay_node = Node(
        package="jzt_robot",
        executable="cmd_vel_relay_node",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
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

    ld = LaunchDescription()
    ld.add_action(declare_gui)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)
    
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)

    ld.add_action(start_robot_state_publisher)
    ld.add_action(spawn_after_gazebo)
    
    ld.add_action(controllers_after_spawn)
    ld.add_action(odom_relay_node)
    ld.add_action(cmd_vel_relay_node)

    return ld