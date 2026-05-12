import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    """MoveIt launch for Gazebo simulation.

    Assumes Gazebo is already running (via flapping_platform.gazebo.launch.py)
    and controllers are spawned. Only starts move_group + rviz2.
    """
    pkg_share_description = get_package_share_directory("flapping_platform_description")
    pkg_share_moveit_config = get_package_share_directory("flapping_platform_moveit_config")

    gazebo_urdf_xacro = os.path.join(
        pkg_share_description, "urdf", "robots", "flapping_platform_gazebo.urdf.xacro"
    )
    initial_positions_file = os.path.join(
        pkg_share_moveit_config, "config", "gazebo", "initial_positions.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("flapping_platform", package_name="flapping_platform_moveit_config")
        .robot_description(
            file_path=gazebo_urdf_xacro,
            mappings={
                "initial_positions_file": initial_positions_file,
                "use_gazebo": "true",
            },
        )
        .robot_description_semantic(
            file_path="config/gazebo/flapping_platform.srdf"
        )
        .trajectory_execution(
            file_path="config/gazebo/moveit_controllers.yaml"
        )
        .robot_description_kinematics(
            file_path="config/gazebo/kinematics.yaml"
        )
        .joint_limits(file_path="config/gazebo/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # ---- move_group ----
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": True,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": ":0"},
    )

    # ---- RViz ----
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/gazebo/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        {"use_sim_time": True},
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld
