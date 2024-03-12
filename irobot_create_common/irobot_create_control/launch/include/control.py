# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from irobot_create_common_bringup.namespace import GetNamespacedName
import os

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
]


def generate_launch_description():
    print('ENTERED CONTROL.PY')
    pkg_create3_control = get_package_share_directory('irobot_create_control')

    namespace = LaunchConfiguration('namespace')

    controller_manager_path = [namespace, '/controller_manager']

    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'control.yaml'])

    diffdrive_controller_node_ns = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', controller_manager_path],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    joint_state_broadcaster_spawner_ns = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', controller_manager_path],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback_ns = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_ns,
            on_exit=[diffdrive_controller_node_ns],
        )
    )

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', 'controller_manager'],
        output='screen',
        condition=LaunchConfigurationEquals('namespace', '')
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
        condition=LaunchConfigurationEquals('namespace', '')

    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(joint_state_broadcaster_spawner_ns)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback_ns)
    ld.add_action(diffdrive_controller_callback)

    return ld