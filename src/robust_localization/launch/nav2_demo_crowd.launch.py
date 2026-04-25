# launch/nav2_demo_crowd.launch.py
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robust_localization')
    
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # 1. 加载地图
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
        
        # 2. 启动鲁棒定位（替代 AMCL）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'robust_localization.launch.py'])
            ),
            launch_arguments={
                'config_file': PathJoinSubstitution([pkg_share, 'config', 'nav2_crowd_params.yaml'])
            }.items()
        ),
        
        # 3. Nav2 其余部分（controller, planner, behavior）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'params_file': PathJoinSubstitution([pkg_share, 'config', 'nav2_crowd_params.yaml']),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),
    ])