"""Launch Atlas squatting in RVIZ

   ros2 launch 133finalproject squat.launch.py

Starts RVIZ, the robot_state_publisher, and the squat node (which
publishes /joint_states and the world->pelvis TF).  No joint GUI, since
it would fight the squat node over /joint_states.
"""

import os
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


def generate_launch_description():
    package = '133finalproject'
    rvizcfg = os.path.join(pkgdir(package), 'rviz/viewsquat.rviz')
    urdf    = os.path.join(pkgdir(package), 'urdf/atlas_v5.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        Node(name='robot_state_publisher', package='robot_state_publisher',
             executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(name='rviz', package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rvizcfg], on_exit=Shutdown()),
        Node(name='squat', package=package, executable='squat', output='screen'),
    ])
