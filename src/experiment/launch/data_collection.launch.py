from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. C++ 采集器：负责稳定写 CSV
        Node(
            package='experiment',
            executable='force_collector',
            name='force_recorder',
            parameters=[{'data_path': '/home/dushi/flapping_platform_ws/src/experiment/data'}],
            output='screen'
        ),
        
        # 2. Python 可视化：负责弹窗画图
        Node(
            package='experiment',
            executable='force_visualizer.py',
            name='force_plotter',
            output='screen'
        )
    ])