"""

Copyright 2024 Bonn-Rhein-Sieg University

Author: Vamsi Kalagaturu

"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():
    declare_joint_state_gui = DeclareLaunchArgument(
        "joint_state_gui",
        default_value="true",
        description="Launch joint state gui publisher",
    )

#  file changed to eddie_gz urdf with latest base and no gripper
    eddie_xacro_file = os.path.join(
        get_package_share_directory("eddie_description"), "robots", "eddie_gz.urdf.xacro"
    )

    eddie_description_config = Command(
        [FindExecutable(name="xacro"), " ", eddie_xacro_file]
    )

    eddie_description = {
        "robot_description": ParameterValue(eddie_description_config, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[eddie_description],
        output="screen",
    )

    # initial positions
    zero_positions_config = os.path.join(
        get_package_share_directory("eddie_description"),
        "config",
        "zero_positions.yaml",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("joint_state_gui")),
        output="screen",
        parameters=[zero_positions_config],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("joint_state_gui")),
        output="screen",
        parameters=[zero_positions_config],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # arguments=["-d", os.path.join(get_package_share_directory("eddie_description"), "rviz", "eddie.rviz")],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_joint_state_gui,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz2_node,
        ]
    )
