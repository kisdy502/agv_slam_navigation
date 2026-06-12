#!/usr/bin/env python3
"""
三舵轮机器人 Gazebo 仿真启动文件
支持 3D LiDAR 仿真
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
    package_name = "triple_steer_robot"
    robot_name_in_model = "tripleSteerRobot01"
    xacro_name = "triple_steer_robot.urdf.xacro"
    gazebo_params_file_name = "gazebo_params.yaml"

    pkg_share = FindPackageShare(package_name).find(package_name)
    xacro_model_path = os.path.join(pkg_share, f"urdf/{xacro_name}")
    gazebo_params_path = os.path.join(pkg_share, f"config/{gazebo_params_file_name}")

    # ---------- Parameters ----------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock topic as time source'
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Launch Gazebo GUI'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='long_channel.world',
        description='Gazebo world file name (located in world/)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    yaw_angle = LaunchConfiguration('yaw')
    world_name = LaunchConfiguration('world_name')

    gazebo_world_path = PathJoinSubstitution([pkg_share, 'world', world_name])

    # ---------- xacro processing ----------
    doc = xacro.process_file(xacro_model_path)
    robot_desc = doc.toprettyxml(indent='  ')

    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp_urdf.write(robot_desc)
    tmp_urdf.close()
    urdf_file_path = tmp_urdf.name

    # 1. Start Gazebo
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

    # 2. Robot State Publisher
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

    # 3. Spawn Robot
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

    # 4. Controller loading
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

    # TODO: Replace with your triple-steer controller
    load_steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "triple_steer_controller",
            "-c", "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 5. Triple-steer kinematics (cmd_vel -> joint commands)
    triple_steer_kinematics = Node(
        package="triple_steer_robot",
        executable="triple_steer_kinematics_node",
        name="triple_steer_kinematics",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Event chain
    spawn_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=start_gzserver,
            on_start=[
                LogInfo(msg="Gazebo server started -> Spawn robot in 3s"),
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg="Spawning robot..."),
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
                LogInfo(msg="Robot spawned -> Load controllers in 3s"),
                TimerAction(
                    period=3.0,
                    actions=[
                        load_joint_state_broadcaster,
                        TimerAction(
                            period=1.0,
                            actions=[load_steer_controller],
                        ),
                    ],
                ),
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)
    ld.add_action(declare_world_name)

    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(spawn_after_gazebo)
    ld.add_action(controllers_after_spawn)
    ld.add_action(triple_steer_kinematics)

    return ld
