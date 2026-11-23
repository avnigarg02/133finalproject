"""Launch robot in RViz."""

import os
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # Use this package's URDF
    urdf = os.path.join(pkgdir('133finalproject'), 'urdf', 'val_valkyrie_D.urdf')

    with open(urdf, 'r') as file:
        robot_description = file.read()

    node_robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    node_joint_state_publisher = Node(
        name='joint_state_publisher_gui',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # Fixed transform so RViz always has a root frame
    node_static_tf = Node(
        name='static_world_to_base',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'pelvis'],
        output='screen',
    )

    rviz_cfg = os.path.join(pkgdir('133finalproject'), 'rviz', 'view.rviz')
    node_rviz = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        node_static_tf,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz,
    ])