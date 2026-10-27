"""Look at the robot in RViz and move its joints by hand.

MANUAL EDIT (not from fusion2URDF) - this file was rewritten.

What was wrong with the generated version:

  * The README promised "The launch file includes joint_state_publisher_gui
    - use the sliders to move joints", but the file never started it. With
    no source of /joint_states, RViz showed the wheels frozen and printed
    "No transform from [backward_left_wheel] to [base_link]".

  * It started a ros2_control_node and spawned joint_state_broadcaster,
    position_controller AND velocity_controller, all active. The last two
    claim the same command interfaces, so the second one to start failed
    with a resource conflict. None of that belongs in a visualisation
    launch file anyway.

  * The namespace argument was broken: PushRosNamespace moved
    /robot_description to /<ns>/robot_description, but rviz/display.rviz
    subscribes to the absolute topic /robot_description, so the model
    silently disappeared. The argument has been removed.

This file is now purely: describe the robot, publish fake joint states,
draw it.

    ros2 launch @PKG@ display.launch.py
    ros2 launch @PKG@ display.launch.py use_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('@PKG@')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    use_gui = LaunchConfiguration('use_gui')
    prefix = LaunchConfiguration('prefix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='true: joint_state_publisher_gui with sliders. '
                        'false: headless joint_state_publisher, all joints '
                        'held at zero.',
        ),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Link/joint name prefix, passed through to xacro.',
        ),

        # /robot_description, built with use_sim:=false so no Gazebo tags.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'prefix': prefix,
            }.items(),
        ),

        # Sliders. This is what actually publishes /joint_states here -
        # there is no hardware and no controller_manager in this launch file.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
