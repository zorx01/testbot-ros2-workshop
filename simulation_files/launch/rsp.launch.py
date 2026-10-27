"""Robot State Publisher only.

MANUAL EDIT (not from fusion2URDF) - new file.

This is the one place that turns testbot.urdf.xacro into the
/robot_description parameter. display.launch.py and sim.launch.py both
include it rather than each running their own xacro command, so there is
exactly one definition of how the robot is built.

    ros2 launch testbot_description rsp.launch.py
    ros2 launch testbot_description rsp.launch.py use_sim_time:=true

use_sim_time does double duty on purpose. It sets the node's clock source
AND it is passed to xacro as use_sim, which decides whether the Gazebo
plugin and sensor blocks are emitted. If you are on simulated time you are
in Gazebo, and if you are in Gazebo you need those blocks.


WHY THIS FILE DOES NOT USE Command(['xacro ', ...])
---------------------------------------------------
The usual idiom is

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

and it works fine for robot_state_publisher and RViz. It does NOT work
here, because of how gazebo_ros2_control starts its controller_manager on
Humble. It reads robot_description back off the parameter server and then
hands it to the new node as a command-line parameter override, roughly

    --ros-args --param robot_description(colon)=<the entire URDF>

rcl parses the right-hand side of a parameter override as YAML. A colon
followed by a space is YAML's key/value separator, so a URDF containing
one anywhere - including inside an XML comment - stops being a valid
scalar and the whole thing dies with

    [gazebo_ros2_control] parser error Couldn't parse parameter override
    rule(colon) '--param robot_description(colon)=<?xml version="1.0" ?> ...

followed by controller_manager never appearing and every spawner hanging
on "waiting for service /controller_manager/load_controller". The error
names the parameter but says nothing about which character upset it.

Verified by bisection against this robot's own URDF: with comments
stripped it parses, with them it does not, and a bare "a(colon) b" is
enough to reproduce it.

The URDF in this package is heavily commented on purpose, and those
comments are worth more than the convenience of Command(). So the xacro is
processed here with the Python API and the comment nodes are removed from
the DOM before the string is published. The source files keep every
comment; only the runtime copy of the parameter is stripped.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _strip_comments(node):
    """Remove every XML comment from a minidom tree, in place."""
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.nodeType == child.ELEMENT_NODE:
            _strip_comments(child)


def has_controller_files(pkg_share):
    """True once the Day 5 controller download has been added and built."""
    return os.path.isfile(os.path.join(pkg_share, 'urdf', 'testbot.ros2_control.xacro'))


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('testbot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'testbot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    doc = xacro.process_file(xacro_file, mappings={
        'use_sim': use_sim_time,
        'prefix': prefix,
        'use_control': str(has_controller_files(pkg_share)).lower(),
    })
    _strip_comments(doc)
    robot_description = doc.toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time.lower() in ('true', '1'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from Gazebo, and build the URDF with '
                        'the Gazebo plugins and sensors included.',
        ),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Prefix for every link and joint name. Useful for '
                        'multi-robot RViz scenes. NOTE a non-empty prefix '
                        'does NOT work in simulation, because the joint '
                        'names in config/ros2_controllers.yaml are fixed.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
