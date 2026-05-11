import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share_description = get_package_share_directory("flapping_platform_description")
    pkg_share_moveit_config = get_package_share_directory("flapping_platform_moveit_config")

    default_urdf_path = os.path.join(
        pkg_share_description, "urdf", "robots", "flapping_platform_real.urdf.xacro"
    )
    default_initial_positions = os.path.join(
        pkg_share_moveit_config, "config", "flapping_platform_real", "initial_positions.yaml"
    )

    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path", default_value=default_urdf_path,
        description="Absolute path to the real robot URDF xacro file"
    )
    declare_initial_positions = DeclareLaunchArgument(
        "initial_positions_file", default_value=default_initial_positions,
        description="Path to initial positions yaml"
    )
    declare_publish_frequency = DeclareLaunchArgument(
        "publish_frequency", default_value="15.0",
        description="Frequency of state publishing"
    )

    robot_description = Command([
        FindExecutable(name="xacro"), " ",
        LaunchConfiguration("urdf_path"), " ",
        "initial_positions_file:=", LaunchConfiguration("initial_positions_file"),
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": LaunchConfiguration("publish_frequency"),
        }],
    )

    return LaunchDescription([
        declare_urdf_path,
        declare_initial_positions,
        declare_publish_frequency,
        robot_state_publisher_node,
    ])
