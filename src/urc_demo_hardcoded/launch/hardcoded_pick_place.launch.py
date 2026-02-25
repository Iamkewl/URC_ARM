from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("urc_demo_hardcoded"),
        "config",
        "hardcoded_pick_place.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameter file with hard-coded waypoints",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo clock",
            ),
            Node(
                package="urc_demo_hardcoded",
                executable="hardcoded_pick_place",
                name="hardcoded_pick_place",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
