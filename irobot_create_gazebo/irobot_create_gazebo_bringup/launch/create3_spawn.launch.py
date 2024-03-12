#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

import os

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName, EnsureString

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories
    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')

    # Paths
    create3_nodes_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_coverage_node_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'create3_coverage_node.launch.py'])
    dock_description_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'dock_description.launch.py'])
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'robot_description.launch.py'])
    rviz2_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'rviz2.launch.py'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    spawn_dock = LaunchConfiguration('spawn_dock')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_name = GetNamespacedName(namespace, 'create3')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    # Calculate dock offset due to yaw rotation
    x_dock = OffsetParser(x, 0.157)
    yaw_dock = OffsetParser(yaw, 3.1416)

    push_namespace = PushRosNamespace(namespace)

    #namespace_str = EnsureString(namespace)
    #x_dock_str = EnsureString(x_dock)
    #y_str = EnsureString(y)
    #z_str = EnsureString(z)
    #yaw_dock_str = EnsureString(yaw_dock)

    #print(f'in create3_spawn launch launch arguments: namespace: {namespace_str}, x: {x_dock_str}, y: {y_str}, z: {z_str}, yaw: {yaw_dock_str}')


    dock_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch_file]),
        condition=IfCondition(spawn_dock),
        # The robot starts docked
        launch_arguments={'x': x_dock, 'y': y, 'z': z, 'yaw': yaw_dock}.items(),)
        #launch_arguments=[
            #('namespace', namespace), ('x', x_dock), ('y', y), ('z', z), ('yaw', yaw_dock)
        #])

    spawn_dock = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_standard_dock',
        arguments=['-entity',
                   dock_name,
                   '-topic',
                   'standard_dock_description',
                   '-x', x_dock,
                   '-y', y,
                   '-z', z,
                   '-Y', yaw_dock],
        output='screen',
        condition=IfCondition(spawn_dock),
    )

    # Create 3 robot model and description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file]),
        #launch_arguments=[
            #('namespace', namespace)
        #]
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_create3',
        arguments=['-entity',
                   robot_name,
                   '-topic',
                   'robot_description',
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-Y', yaw],
        output='screen'
    )


    # Create 3 nodes
    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch_file]),
        launch_arguments=[
            ('namespace', namespace)
        ]
    )

    # Create 3 coverage
    create3_coverage_node= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_coverage_node_launch_file]),
        launch_arguments=[
            ('namespace', namespace)
        ]
    )

    # RVIZ2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz2_launch_file]),
        condition=IfCondition(use_rviz),
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # add namespace
    ld.add_action(push_namespace)
    # Include robot description
    ld.add_action(robot_description)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_dock)
    ld.add_action(dock_description)
    # Include Create 3 nodes
    ld.add_action(create3_nodes)
    #ld.add_action(create3_coverage_node)
    # Rviz
    ld.add_action(rviz2)
    return ld