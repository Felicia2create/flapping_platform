import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # moveit_configs = MoveItConfigsBuilder("rml_63_description", package_name="rm_63_config").to_moveit_configs()
    moveit_configs = (
        MoveItConfigsBuilder("rml_63_description", package_name="rm_63_config")
        .robot_description(
            file_path="config/rml_63_6fb_description.urdf.xacro", 
            mappings={
                "link6_type": "Link6_6fb", 
                "base_type": "base_link_III"
            }
        )
        .to_moveit_configs()
    )
    # 控制节点
    control_node = Node(
        package="experiment",
        executable="experiment_control",
        output="screen",
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            {"use_sim_time": False},# 仿真环境下设为 True
        ],
    )

    gui_node = Node(
        package="experiment",
        executable="experiment_gui.py",
        output="screen",
    )
    return LaunchDescription([control_node,gui_node])