from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_global_planner',
            executable='astar_planner_node',
            name='astar_planner',
            parameters=[{
                'allow_diagonal': False,
                'heuristic': 'manhattan',
                'straight_cost': 1.0,
                'diag_cost': 1.414,
                'start_x': 1.0,
                'start_y': 0.8,
            }],
            output='screen'
        ),
        # 可选：启动 RViz2 并加载配置
        ExecuteProcess(
            cmd=['rviz2', '-d', '$(find-pkg-share agv_global_planner)/config/astar.rviz'],
            output='screen'
        )
    ])