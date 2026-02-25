import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cleanup_stale_processes(context, *args, **kwargs):
    clean_start = LaunchConfiguration("clean_start").perform(context).strip().lower()
    if clean_start not in {"1", "true", "yes", "on"}:
        return []

    subprocess.run(
        [
            "bash",
            "-lc",
            "pkill -f 'ign gazebo server|ign gazebo gui|ros_gz_bridge/parameter_bridge|arm_wave_motion' || true",
        ],
        check=False,
    )
    return []


def generate_launch_description() -> LaunchDescription:
    motion_params = LaunchConfiguration("motion_params")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")
    default_motion_params = os.path.join(
        get_package_share_directory("urc_demo_hardcoded"),
        "config",
        "arm_wave_motion.yaml",
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

    motion_node = Node(
        package="urc_demo_hardcoded",
        executable="arm_wave_motion",
        name="arm_wave_motion",
        output="screen",
        parameters=[motion_params, {"use_sim_time": False}],
    )

    cleanup_stale = OpaqueFunction(function=_cleanup_stale_processes)

    delayed_stack_start = TimerAction(
        period=1.0,
        actions=[complete_bringup, motion_node],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_params",
                default_value=default_motion_params,
                description="Path to arm motion parameters yaml",
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
                default_value="-r /root/colcon_ws/install/complete_description/share/complete_description/worlds/pick_place_demo.sdf",
                description="Arguments passed to ros_gz_sim gz_sim.launch.py",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="complete_arm",
                description="Spawned robot entity name",
            ),
            DeclareLaunchArgument(
                "clean_start",
                default_value="true",
                description="Kill stale Gazebo/bridge/motion processes before starting",
            ),
            cleanup_stale,
            delayed_stack_start,
        ]
    )
