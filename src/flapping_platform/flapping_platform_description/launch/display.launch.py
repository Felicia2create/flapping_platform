import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'flapping_platform_description'
    
    # 默认路径
    default_model_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'robots',
        'flapping_platform_gazebo.urdf.xacro'
    )

    # 声明参数
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to xacro file'
    )

    # 核心：使用 ParameterValue 包装 Command
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    return LaunchDescription([
        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])