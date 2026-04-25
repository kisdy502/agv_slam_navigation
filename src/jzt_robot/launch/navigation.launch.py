#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer定位 + RViz 可视化启动文件

使用方法:
    # 终端1: 先启动gazebo
    ros2 launch jzt_robot gazebo_diff.launch.py

    # 终端2: 再启动导航（默认 MPPI 控制器）
    ros2 launch jzt_robot navigation.launch.py \
        pbstream_file:=/home/kisdy/maps/cartographer_map.pbstream

    # 使用 DWB 控制器
    ros2 launch jzt_robot navigation.launch.py \
        pbstream_file:=/home/kisdy/maps/cartographer_map.pbstream \
        controller:=dwb
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')
    rviz_config = os.path.join(pkg_share, 'rviz', 'common_nav2.rviz')
    
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    
    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='/home/kisdy/maps/cartographer_map.pbstream',
            description='Cartographer pbstream地图文件'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='localization_2d.lua',
            description='Cartographer Lua配置文件，导航定位用localization_2d.lua'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'param', 'nav2_params_dwb_cartographer.yaml'),
            description='Nav2参数文件路径，默认MPPI，可指定DWB: $(find-pkg-prefix jzt_robot)/share/jzt_robot/param/nav2_params_dwb_cartographer.yaml'
        ),
    ]

    # Cartographer 定位节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
            # '-start_trajectory_with_default_topics', 'false',
            '--ros-args',
            '--log-level', 'WARN',          # 只显示 ERROR 和 FATAL
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu'),  # 添加这行！
        ],
    )
    
    # 占据栅格地图发布
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
            'frame_id': 'map',  # 添加 frame_id
        }],
    )
    
    # Nav2 导航 (只用 navigation_launch.py，不需要 bringup 的 map_server/AMCL)
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    
    # RViz 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # joy 手柄驱动
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'autorepeat_rate': 20.0,
        }],
    )

    # Gamepad 遥控节点
    gamepad_teleop_node = Node(
        package='jzt_robot',
        executable='gamepad_teleop_node',
        name='gamepad_teleop_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Nav2 导航模式启动']),
        LogInfo(msg=['地图: $(var pbstream_file)']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[occupancy_grid_node]),
        TimerAction(period=3.0, actions=[nav2_launch]),
        TimerAction(period=3.5, actions=[joy_node]),
        TimerAction(period=4.0, actions=[rviz_node, gamepad_teleop_node]),

        LogInfo(msg=['导航节点 + 手柄遥控 + RViz 已启动']),
        LogInfo(msg=['在RViz中设置2D Goal启动自主导航，或使用手柄手动控制']),
    ])
