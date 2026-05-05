import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = "jzt_robot"

    # 可传入参数声明
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='jztDiffRobot',
        description='Name of the robot model in simulation'
    )
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='jzt_work_room.world',
        description='World file name'
    )
    declare_urdf_name = DeclareLaunchArgument(
        'urdf_name',
        default_value='jzt_diff.urdf',
        description='URDF file name'
    )
    declare_gazebo_params_file = DeclareLaunchArgument(
        'gazebo_params_file',
        default_value='gazebo_params.yaml',
        description='Gazebo parameters YAML file'
    )
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0', description='Robot X position')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Robot Y position')
    declare_z = DeclareLaunchArgument('z', default_value='0.0', description='Robot Z position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Robot Yaw angle')

    # 获取配置值（延迟求值）
    robot_name_in_model = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    urdf_name = LaunchConfiguration('urdf_name')
    gazebo_params_file_name = LaunchConfiguration('gazebo_params_file')
    gui = LaunchConfiguration('gui')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    yaw_angle = LaunchConfiguration('yaw')

    # 包路径
    pkg_share = FindPackageShare(package_name).find(package_name)

    # 使用 PathJoinSubstitution 延迟拼接路径
    urdf_model_path = PathJoinSubstitution([pkg_share, 'urdf', urdf_name])
    gazebo_world_path = PathJoinSubstitution([pkg_share, 'world', world_name])
    gazebo_params_path = PathJoinSubstitution([pkg_share, 'config', gazebo_params_file_name])

    # 1. 启动 gzserver
    start_gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path,
            '--ros-args',
            '--params-file', gazebo_params_path,
        ],
        output='screen',
        shell=False,
    )

    # 2. 启动 gzclient
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    # 3. 生成机器人
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name_in_model,
            "-file", urdf_model_path,
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
            "-Y", yaw_angle,
            "-robot_namespace", "/",
        ],
        output="screen",
    )

    # 4. Robot State Publisher（动态读取 URDF）
    start_robot_state_publisher_cmd = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[
        {"robot_description": ParameterValue(Command(['cat ', urdf_model_path]), value_type=str)},
        {"use_sim_time": True}
    ],
)


    ld = LaunchDescription()
    # 添加参数声明
    ld.add_action(declare_robot_name)
    ld.add_action(declare_world_name)
    ld.add_action(declare_urdf_name)
    ld.add_action(declare_gazebo_params_file)
    ld.add_action(declare_gui)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)
    # 添加动作
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld