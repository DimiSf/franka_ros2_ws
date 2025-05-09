#!/usr/bin/env -S ros2 launch

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_robot_description(context: LaunchContext):
    xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro"
    )

    robot_desc = xacro.process_file(
        xacro_file,
        mappings={
            "hand": "true",
            "ros2_control": "true",
            "gazebo": "true",
            "gazebo_effort": "true",
            "robot_ip": "dont-care",
            "use_fake_hardware": "true",
            "fake_sensor_commands": "true"
        }
    ).toxml()

    # Wrap in robot_state_publisher node
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_desc}]
        )
    ]


def generate_launch_description():
    # Set Ignition resource path
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(
        get_package_share_directory("franka_description")
    )

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ign_verbosity", default_value="3"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        DeclareLaunchArgument("world", default_value="empty.sdf")
    ]

    # Launch Ignition Gazebo
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": LaunchConfiguration("world")
        }.items()
    )

    # Manually process xacro and publish robot_description
    robot_description_node = OpaqueFunction(function=load_robot_description)

    # Spawn in Gazebo
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen"
    )

    # Clock bridge
    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="log"
    )

    # Include MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_fr3_moveit_config"),
                "launch",
                "moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "robot_ip": "dont-care",
            "use_fake_hardware": "true",
            "fake_sensor_commands": "true"
        }.items()
    )

    return LaunchDescription(
        declared_arguments + [
            ign_gz,
            robot_description_node,
            spawn,
            bridge,
            moveit_launch
        ]
    )
