#!/usr/bin/env python3
# Authors: Deebul Nair

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    world = os.path.join(
        get_package_share_directory('robile_gazebo'),
        'worlds',
        'empty_scene.world'
    )

    robile_nav_dir = get_package_share_directory("robile_navigation")
    map_name = "closed_walls_map"
    map_file = os.path.join(robile_nav_dir, "maps", map_name + ".yaml")
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_file},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    tf2_ros_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "gazebo",
                    "gazebo_robile_laserscanner_camera.xacro"
                ]
            ),
            " ",
            "platform_config:=4_wheel_config",
            " ",
            "movable_joints:=False",
        ]
    )

    def create_robot_spawn_group(namespace, x_pose, y_pose):
        return GroupAction([
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': ParameterValue(robot_description_content, value_type=str)
                }],
                namespace=namespace,
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                namespace=namespace,
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', f'/{namespace}/robot_description',
                           '-entity', namespace,
                           '-x', str(x_pose),
                           '-y', str(y_pose),
                           '-z', '0.0'
                           ],
                output='screen'
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0",
                           "0", "base_footprint", "base_link"],
                namespace=namespace,
            ),
            LogInfo(msg=f"Spawning robot {namespace} at ({x_pose}, {y_pose})")
        ])

    def generate_robot_descriptions(robot_count):
        robot_spawning_cmds = []
        for i in range(robot_count):
            namespace = f'robile_{i}'
            x_pose = float(i) * 2.0  # Space robots 2 meters apart
            y_pose = 0.0
            robot_spawning_cmds.append(create_robot_spawn_group(namespace, x_pose, y_pose))
        return robot_spawning_cmds

    robots = generate_robot_descriptions(2)

    rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_use_rviz_cmd,
        rviz_cmd,
        gzserver_cmd,
        gzclient_cmd,
        *robots,
        # map_server,
        # lifecycle_manager,
        tf2_ros_map_to_odom,
    ])
