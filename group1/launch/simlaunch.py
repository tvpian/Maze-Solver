#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazebo and TurtleBot3 launch setup
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'maze_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    # Node definitions
    aruco_node_cmd = Node(
        package='ros2_aruco',
        executable='aruco_node',
        output='log',
        parameters=[{'use_sim_time': True}]
    )

    tb3_aruco_detector_bcaster_node_cmd = Node(
        package='group1',
        executable='tb3_aruco_broadcaster_node',
        output='log',
        parameters=[{'use_sim_time': True}]
    )

    tb3_parts_broadcaster_cmd = Node(
        package='group1',
        executable='tb3_parts_broadcaster_node',
        output='log',
        parameters=[{'use_sim_time': True}]
    )

    listener_demo_dcomp_cmd = Node(
        package='group1',
        executable='tb3_aruco_listener_node',
        output='log',
        parameters=[
            {'use_sim_time': True},
            '--params-file /home/rwa3_ws/src/group1/config/params.yaml'
        ]
    )

    tb3_parts_listener_cmd = Node(
        package='group1',
        executable='tb3_parts_listener_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Launch description
    ld = LaunchDescription()

    # Add Gazebo and TurtleBot3 commands
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    # Add custom nodes
    ld.add_action(aruco_node_cmd)
    ld.add_action(tb3_aruco_detector_bcaster_node_cmd)
    ld.add_action(tb3_parts_broadcaster_cmd)
    ld.add_action(listener_demo_dcomp_cmd)
    ld.add_action(tb3_parts_listener_cmd)

    return ld
