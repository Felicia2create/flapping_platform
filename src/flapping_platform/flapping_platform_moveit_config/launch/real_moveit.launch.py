import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    """Real robot MoveIt launch (no ros2_control).

    Includes only move_group + rviz2.
    rm_driver, rm_control, and robot_state_publisher are launched separately
    via flapping_platform_bringup.
    """
    pkg_share_description = get_package_share_directory("flapping_platform_description")
    pkg_share_moveit_config = get_package_share_directory("flapping_platform_moveit_config")

    real_urdf_xacro = os.path.join(
        pkg_share_description, "urdf", "robots", "flapping_platform_real.urdf.xacro"
    )
    initial_positions_file = os.path.join(
        pkg_share_moveit_config, "config", "flapping_platform_real", "initial_positions.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("flapping_platform", package_name="flapping_platform_moveit_config")
        .robot_description(
            file_path=real_urdf_xacro,
            mappings={
                "initial_positions_file": initial_positions_file,
            },
        )
        .robot_description_semantic(
            file_path="config/flapping_platform_real/flapping_platform_real.srdf"
        )
        .trajectory_execution(
            file_path="config/flapping_platform_real/moveit_controllers_real.yaml"
        )
        .robot_description_kinematics(
            file_path="config/flapping_platform_real/kinematics.yaml"
        )
        .joint_limits(file_path="config/flapping_platform_real/joint_limits.yaml")
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
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        trajectory_execution,
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
            default_value=str(moveit_config.package_path / "config/flapping_platform/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
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
