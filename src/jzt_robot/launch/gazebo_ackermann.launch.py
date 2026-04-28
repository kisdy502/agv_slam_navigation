import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = "jzAckermannRobot01"
    package_name = "jzt_robot"
    world_name = "jz_house.world"
    urdf_name = "jzt_ackermann_deepseek.urdf"
    gazebo_params_file_name="gazebo_params.yaml"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    gazebo_world_path = os.path.join(pkg_share, f"world/{world_name}")
    gazebo_params_path = os.path.join(pkg_share,  f"config/{gazebo_params_file_name}")


    # 声明参数
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='Robot X position')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Robot Y position')
    declare_z = DeclareLaunchArgument('z', default_value='0.0', description='Robot Z position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Robot Yaw angle')

    gui = LaunchConfiguration('gui')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    yaw_angle = LaunchConfiguration('yaw')

    # ========== 关键：分开启动 gzserver 和 gzclient ==========
    
    # 1. 启动 gzserver（后台物理引擎，支持 ROS 参数）
    start_gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path,
            '--ros-args',
            '--params-file', gazebo_params_path,  # ✅ 使用参数文件
        ],
        output='screen',
        shell=False,
    )

    # 2. 启动 gzclient（GUI前端，纯显示，不需要 ROS 参数）
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    # 3. 延迟启动 gzclient，等 gzserver 启动完成
    # 或者直接用 condition，launch 会并行执行

    # Spawn robot
    spawn_entity_cmd = Node(
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

    # Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": open(urdf_model_path, 'r').read()},
            {"use_sim_time": True}
        ],
    )

    ld.add_action(declare_gui)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)
   
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld