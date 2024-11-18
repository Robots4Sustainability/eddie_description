from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    
    eddie_xacro_file = os.path.join(
        get_package_share_directory("eddie_description"), "urdf", "eddie_robot.urdf.xacro"
    )
    
    eddie_rviz_config_file = os.path.join(
        get_package_share_directory("eddie_description"), "config/rviz", "eddie.rviz"
    )
    
    eddie_description_config = Command(
        [FindExecutable(name="xacro"), " ", eddie_xacro_file]
    )

    eddie_description = {
        "robot_description": ParameterValue(eddie_description_config, value_type=str)
    }
    
    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[eddie_description]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', eddie_rviz_config_file]
        )
    ])
