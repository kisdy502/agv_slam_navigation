"""Gamepad Teleop Launch File

启动游戏手柄遥控节点:
1. joy_node - 读取手柄硬件，发布 /joy topic
2. gamepad_teleop_node - 将手柄输入映射为 /cmd_vel

使用方法:
    ros2 launch jzt_robot gamepad_teleop.launch.py

配合gazebo仿真使用:
    终端1: ros2 launch jzt_robot gazebo.launch.py
    终端2: ros2 launch jzt_robot gamepad_teleop.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'jzt_robot'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # 配置文件路径
    config_file = os.path.join(pkg_dir, 'config', 'gamepad_config.yaml')
    
    return LaunchDescription([
        # 1. joy_node - 手柄驱动，读取 /dev/input/js0
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,              # /dev/input/js0
                'autorepeat_rate': 20.0,     # 按键自动重复频率 Hz
                'coalesce_interval_ms': 10,   # 事件合并间隔 ms
            }],
        ),
        
        # 2. gamepad_teleop_node - 手柄遥控逻辑
        Node(
            package='jzt_robot',
            executable='gamepad_teleop_node',
            name='gamepad_teleop_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
