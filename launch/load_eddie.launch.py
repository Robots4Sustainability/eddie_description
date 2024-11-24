from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    # use ros2 control arg
    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Use ros2 control",
    )

    # use gazebo simulation arg
    use_gz_sim_arg = DeclareLaunchArgument(
        "use_gz_sim",
        default_value="false",
        description="Use gazebo simulation",
    )

    # use isaac sim arg
    use_isaac_sim_arg = DeclareLaunchArgument(
        "use_isaac_sim_arg",
        default_value="false",
        description="Use Isaac sim",
    )

    # use fake hardware arg
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use fake hardware",
    )

    # use fake sensor commands arg
    use_fake_sensor_commands_arg = DeclareLaunchArgument(
        "use_fake_sensor_commands",
        default_value="false",
        description="Use fake sensor commands",
    )

    eddie_xacro_file = os.path.join(
        get_package_share_directory("eddie_description"), "urdf", "eddie_robot.urdf.xacro"
    )

    eddie_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            eddie_xacro_file,
            " ",
            "use_ros2_control:=",
            LaunchConfiguration("use_ros2_control"),
            " ",
            "use_gz_sim:=",
            LaunchConfiguration("use_gz_sim"),
        ]
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

    return LaunchDescription(
        [
            use_ros2_control_arg,
            use_gz_sim_arg,
            use_isaac_sim_arg,
            use_fake_hardware_arg,
            use_fake_sensor_commands_arg,
            robot_state_publisher_node,
        ]
    )
