import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # use ros2 control arg
    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Use ros2 control",
    )

    # use kelo_tulip arg
    use_use_kelo_tulip_arg = DeclareLaunchArgument(
        "use_kelo_tulip",
        default_value="false",
        description="Use kelo_tulip driver to control wheels",
    )
    
    # use gazebo simulation arg
    use_gz_sim_arg = DeclareLaunchArgument(
        "use_gz_sim",
        default_value="false",
        description="Use gazebo simulation",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    # use isaac sim arg
    use_isaac_sim_arg = DeclareLaunchArgument(
        "use_isaac_sim",
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
        get_package_share_directory("eddie_description"),
        "urdf",
        "eddie_robot.urdf.xacro",
    )

    eddie_description_urdf = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            eddie_xacro_file,
            " ",
            "use_ros2_control:=",
            LaunchConfiguration("use_ros2_control"),
            " ",
            "use_kelo_tulip:=",
            LaunchConfiguration("use_kelo_tulip"),            
            " ",
            "use_gz_sim:=",
            LaunchConfiguration("use_gz_sim"),
            " ",
            "use_sim_isaac:=",
            LaunchConfiguration("use_isaac_sim"),
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " ",
            "use_fake_sensor_commands:=",
            LaunchConfiguration("use_fake_sensor_commands"),
        ]
    )
    
    use_kelo_tulip = LaunchConfiguration("use_kelo_tulip")
    log_use_kelo_tulip = LogInfo(msg=["use_kelo_tulip: ", use_kelo_tulip])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": eddie_description_urdf,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_ros2_control_arg,
            use_use_kelo_tulip_arg,
            use_gz_sim_arg,
            use_isaac_sim_arg,
            use_fake_hardware_arg,
            use_fake_sensor_commands_arg,
            robot_state_publisher_node,
            log_use_kelo_tulip,
        ]
    )
