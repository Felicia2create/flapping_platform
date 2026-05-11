import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from srdfdom.srdf import SRDF


def generate_launch_description():
    """Self-contained demo for flapping_platform real robot (mock_components).

    Includes:
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    # Paths
    pkg_share_description = get_package_share_directory("flapping_platform_description")
    pkg_share_moveit_config = get_package_share_directory("flapping_platform_moveit_config")

    real_urdf_xacro = os.path.join(
        pkg_share_description, "urdf", "robots", "flapping_platform_real.urdf.xacro"
    )
    initial_positions_file = os.path.join(
        pkg_share_moveit_config, "config", "flapping_platform_real", "initial_positions.yaml"
    )

    # MoveIt configuration — loads real URDF (unprefixed joints) via xacro
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

    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # Virtual joint static TF (from SRDF)
    generate_static_virtual_joint_tfs_launch(ld, moveit_config)

    # Robot State Publisher
    generate_rsp_launch(ld, moveit_config)

    # MoveGroup
    generate_move_group_launch(ld, moveit_config)

    # RViz
    generate_moveit_rviz_launch(ld, moveit_config)

    # Warehouse DB
    db_config = LaunchConfiguration("db")
    ld.add_action(
        Node(
            package="warehouse_ros_mongo",
            executable="mongo_wrapper_ros.py",
            parameters=[
                {"warehouse_port": 33829},
                {"warehouse_host": "localhost"},
                {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
            ],
            output="screen",
            condition=IfCondition(db_config),
        )
    )

    # ros2_control_node with mock_components
    ros2_controllers_path = os.path.join(
        pkg_share_moveit_config, "config", "flapping_platform_real", "ros2_controllers.yaml"
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, ros2_controllers_path],
        )
    )

    # Controller spawners
    generate_spawn_controllers_launch(ld, moveit_config)

    return ld


def generate_static_virtual_joint_tfs_launch(ld, moveit_config):
    name_counter = 0
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id", vj.parent_frame,
                        "--child-frame-id", vj.child_link,
                    ],
                )
            )
            name_counter += 1
    return ld


def generate_rsp_launch(ld, moveit_config):
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
        ],
    )
    ld.add_action(rsp_node)
    return ld


def generate_move_group_launch(ld, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
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
    return ld


def generate_moveit_rviz_launch(ld, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
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


def generate_spawn_controllers_launch(ld, moveit_config):
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster", "turntable_velocity_controller"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld
