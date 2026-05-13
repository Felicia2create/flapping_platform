import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Bringup for flapping_platform real robot.

    Launches all components of the real robot control pipeline:

    rm_driver --> /arm_joint_states --\
                                       --> joint_state_merger --> /joint_states
    /turntable_angle -----------------/                            |
                                                                   +--> robot_state_publisher
                                                                   +--> move_group

    move_group --> /rm_group_controller/follow_joint_trajectory
                      |
                      v
                rm_control (cubic spline interpolation)
                      |
                      v
                /rm_driver/movej_canfd_cmd --> rm_driver --> TCP --> hardware
    """

    # ---- rm_driver (TCP <-> hardware) ----
    # 使用 Node + remappings 代替 IncludeLaunchDescription，将 rm_driver 的
    # joint_states 输出重定向到 arm_joint_states，供 joint_state_merger 合并
    # rm_63_driver.launch.py 仅包含单个 Node，展开后逻辑等价
    rm_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        parameters=[os.path.join(
            get_package_share_directory("rm_driver"),
            "config", "rm_63_config.yaml",
        )],
        output="screen",
        remappings=[("joint_states", "arm_joint_states")],
    )

    # ---- rm_control (FollowJointTrajectory action server) ----
    rm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rm_control"),
            "launch", "rm_63_control.launch.py",
        ))
    )

    # ---- robot_state_publisher (our URDF with turntable) ----
    robot_state_publisher_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("flapping_platform_description"),
            "launch", "robot_state_publisher_real.launch.py",
        ))
    )

    # ---- joint_state_merger (arm joints + turntable -> /joint_states) ----
    joint_state_merger = Node(
        package="flapping_platform_sensor_interface",
        executable="joint_state_merger.py",
        name="joint_state_merger",
        output="screen",
    )

    # ---- MoveIt + RViz ----
    real_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("flapping_platform_moveit_config"),
            "launch", "real_moveit.launch.py",
        ))
    )

    return LaunchDescription([
        rm_driver,
        rm_control,
        joint_state_merger,
        robot_state_publisher_real,
        real_moveit,
    ])
