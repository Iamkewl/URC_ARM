import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("complete_description")
    default_model = os.path.join(package_share, "urdf", "Complete.xacro")
    default_rviz = os.path.join(package_share, "launch", "urdf.rviz")

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description = {"robot_description": Command(["xacro ", model])}

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
