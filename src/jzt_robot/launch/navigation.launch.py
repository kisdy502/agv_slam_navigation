#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer定位 + RViz 可视化启动文件

使用方法:
    # 启动示例（默认参数）
    ros2 launch jzt_robot navigation.launch.py

    # 指定地图和配置
    ros2 launch jzt_robot navigation.launch.py \
        pbstream_file:=/home/kisdy/maps/jz_map.pbstream

    # 阿克曼底盘 (默认)
    ros2 launch jzt_robot navigation.launch.py \
        cmd_topic:=/ackermann_steering_controller/reference_unstamped

    # 差速底盘 / 麦克纳姆底盘
    ros2 launch jzt_robot navigation.launch.py \
        cmd_topic:=/cmd_vel

    # 真机调试
    ros2 launch jzt_robot navigation.launch.py \
        use_sim_time:=false
        
    # 完整
    ros2 launch jzt_robot navigation.launch.py \
        pbstream_file:=/home/kisdy/maps/jz_map.pbstream \
        cmd_topic:=/ackermann_steering_controller/reference_unstamped \
        use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# import logging

# logging.getLogger().setLevel(logging.DEBUG)

def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')
    rviz_config = os.path.join(pkg_share, 'rviz', 'common_nav2.rviz')
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(pkg_share,'param','nav2_params_mppi_cartographer_ackermann.yaml'))
    default_nav2_param = os.path.join(pkg_share, 'param', 'nav2_params_mppi_cartographer_ackermann.yaml')

    cartographer_config_dir = os.path.join(pkg_share, 'config')
    
    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='/home/kisdy/maps/jz_map.pbstream',
            description='Cartographer pbstream地图文件'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='localization_2d.lua',
            description='Cartographer Lua配置文件，导航定位用localization_2d.lua'
        ),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to param file to load'),
        # 新增：遥控器控制的速度话题
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/ackermann_steering_controller/reference_unstamped',
            description='手柄遥控器发布的速度话题 (阿克曼: /ackermann_steering_controller/reference_unstamped, 差速/麦克纳姆: /cmd_vel)'
        ),

        # 新增：是否使用仿真时间
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用 /clock 话题作为时间源（仿真为 true，真机为 false）'
        ),
    ]
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_topic = LaunchConfiguration('cmd_topic')

    # Cartographer 定位节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
            'use_sim_time': use_sim_time,
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
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )
    
    # RViz 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # joy 手柄驱动
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 1,
            'autorepeat_rate': 20.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # Gamepad 遥控节点
    gamepad_teleop_node = Node(
        package='jzt_robot',
        executable='gamepad_teleop_node',
        name='gamepad_teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': cmd_topic,       # 从启动参数传入
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Nav2 导航模式启动']),
        LogInfo(msg=['地图: $(var pbstream_file)']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=0.2, actions=[cartographer_node]),
        TimerAction(period=3.0, actions=[occupancy_grid_node]),
        TimerAction(period=6.0, actions=[nav2_launch]),
        TimerAction(period=9.0, actions=[joy_node]),
        TimerAction(period=11.0, actions=[rviz_node]),
        TimerAction(period=12.0, actions=[gamepad_teleop_node]),

        LogInfo(msg=['导航节点 + 手柄遥控 + RViz 已启动']),
        LogInfo(msg=['在RViz中设置2D Goal启动自主导航，或使用手柄手动控制']),
    ])
