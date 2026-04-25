# launch/robust_localization.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value='adaptive_ekf.yaml'),
        
        Node(
            package='robust_localization',
            executable='robust_localization_node',
            name='robust_localization',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('scan_front', '/scan_front'),
                ('scan_rear', '/scan_rear'),
                ('odom', '/odom'),
                ('map', '/map'),
                ('initialpose', '/initialpose'),
                ('robust_pose', '/robust_pose'),
                ('degeneracy_status', '/degeneracy_status'),
            ]
        ),
    ])