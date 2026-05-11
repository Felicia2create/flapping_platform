import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Bringup for flapping_platform real robot.

    Launches all components of the real robot control pipeline:

    rm_driver ──► /joint_states ──► robot_state_publisher (our URDF + turntable)
             ──► /joint_states ──► move_group (monitoring)

    move_group ──► /rm_group_controller/follow_joint_trajectory
                      │
                      ▼
                rm_control (cubic spline interpolation)
                      │
                      ▼
                /rm_driver/movej_canfd_cmd ──► rm_driver ──► TCP ──► hardware
    """

    # ---- rm_driver (TCP ↔ hardware) ----
    rm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("rm_driver"),
            "launch", "rm_63_driver.launch.py",
        ))
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
        robot_state_publisher_real,
        real_moveit,
    ])
