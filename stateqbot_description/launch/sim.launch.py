"""Spawn stateqbot in Gazebo Classic with diff_drive_controller running.

MANUAL EDIT (not from fusion2URDF) - new file.

    ros2 launch stateqbot_description sim.launch.py
    ros2 launch stateqbot_description sim.launch.py rviz:=true
    ros2 launch stateqbot_description sim.launch.py world:=/path/to/other.world

Then drive it:

    ros2 topic pub --rate 10 \
        /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
        "{linear: {x: 0.15}}"

Startup order matters and is enforced with event handlers rather than
timers. A `TimerAction` that waits "long enough" is the usual quick fix and
the usual source of flaky launches on a loaded machine:

    gazebo -> spawn_entity -> joint_state_broadcaster -> diff_drive_controller

joint_state_broadcaster must be active before diff_drive_controller,
otherwise there is a window where /joint_states is silent and the
odometry integrator starts from a stale position.

NOTE: Gazebo Classic reached end of life in January 2025. It is used here
because this workshop targets ROS 2 Humble, where Classic is the supported
pairing; new projects should use modern Gazebo (gz-sim) with ros_gz.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gazebo_prefix():
    """Find the installed Gazebo Classic share directory, e.g. gazebo-11.

    Sourcing /opt/ros/humble/setup.bash does NOT set GAZEBO_MODEL_PATH or
    GAZEBO_RESOURCE_PATH - only /usr/share/gazebo/setup.sh does, and
    nothing obliges a student to have sourced it. Without them Gazebo
    cannot find its own media and prints
        [Err] [RTShaderSystem.cc:480] Unable to find shader lib.
    and falls back to the online model database for anything included by
    model:// URI. Rather than require the extra source line, the launch
    file works out the path and sets the variables itself.
    """
    candidates = sorted(glob.glob('/usr/share/gazebo-*'), reverse=True)
    return candidates[0] if candidates else ''


def generate_launch_description():
    pkg_share = get_package_share_directory('stateqbot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'obstacles.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'sim.rviz')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    use_gui = LaunchConfiguration('gui')

    # ---- Gazebo ---------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            # gui:=false runs gzserver only. Physics, controllers and every
            # topic still work; you just get no window. Much faster, and the
            # only way to run this on a machine with no display.
            'gui': use_gui,
        }.items(),
    )

    # ---- /robot_description, built WITH the Gazebo tags ------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ---- Put the robot in the world -------------------------------------
    # -z 0.03 drops it ~1 cm above the ground (the wheels reach to
    # z = -0.0195 in base_link) so it settles instead of starting
    # interpenetrated with the ground plane, which makes ODE launch it.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'stateqbot',
            '-z', '0.03',
        ],
    )

    # ---- Controllers ----------------------------------------------------
    # The controller_manager lives inside the Gazebo process (started by
    # libgazebo_ros2_control), so these spawners talk to /controller_manager
    # but no ros2_control_node is launched here.
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )

    load_diff_drive = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'diff_drive_controller'],
        output='screen',
    )

    # Loaded but NOT activated: it would fight diff_drive_controller for the
    # same velocity interfaces. Used by the controller-lifecycle demo.

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    gazebo_prefix = _gazebo_prefix()
    gazebo_env = []
    if gazebo_prefix:
        gazebo_env = [
            AppendEnvironmentVariable(
                'GAZEBO_MODEL_PATH', os.path.join(gazebo_prefix, 'models')),
            AppendEnvironmentVariable(
                'GAZEBO_RESOURCE_PATH', gazebo_prefix),
        ]

    # Without the controller files the wheels have no joint_states source,
    # so publish them at zero: the TF tree is complete and RViz shows the robot.
    if os.path.isfile(os.path.join(pkg_share, 'urdf', 'stateqbot.ros2_control.xacro')):
        after_spawn = [
            RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])),
            RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_diff_drive])),
            RegisterEventHandler(OnProcessExit(target_action=load_diff_drive,
                                               on_exit=[rviz])),
        ]
    else:
        after_spawn = [
            Node(package='joint_state_publisher', executable='joint_state_publisher',
                 parameters=[{'use_sim_time': True}]),
            # The robot cannot move yet, so odom sits on base_link (sim.rviz uses odom).
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
                 parameters=[{'use_sim_time': True}]),
            RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[rviz])),
        ]

    return LaunchDescription(gazebo_env + [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start RViz with rviz/sim.rviz.',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false runs gzserver headless, without the Gazebo '
                        'window.',
        ),

        gazebo,
        rsp,
        spawn_entity,
    ] + after_spawn)
