import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")

    default_params = os.path.join(
        get_package_share_directory("urc_demo_hardcoded"),
        "config",
        "hardcoded_pick_place.yaml",
    )

    complete_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("complete_description"),
                "launch",
                "gazebo_bringup.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "world": world,
            "entity_name": entity_name,
        }.items(),
    )

    pick_place_node = Node(
        package="urc_demo_hardcoded",
        executable="hardcoded_pick_place",
        name="hardcoded_pick_place",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to hardcoded pick/place parameter yaml",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=(
                    "/root/colcon_ws/install/complete_description/share/"
                    "complete_description/worlds/pick_place_demo.sdf"
                ),
                description="Fortress world SDF path",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="-s -r /root/colcon_ws/install/complete_description/share/complete_description/worlds/pick_place_demo.sdf",
                description="Arguments passed to ros_gz_sim gz_sim.launch.py",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="complete_arm",
                description="Spawned robot entity name",
            ),
            complete_bringup,
            pick_place_node,
        ]
    )
