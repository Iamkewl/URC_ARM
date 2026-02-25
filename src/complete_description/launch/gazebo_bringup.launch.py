import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("complete_description")
    xacro_file = os.path.join(package_share, "urdf", "Complete.xacro")
    default_world = os.path.join(package_share, "worlds", "pick_place_demo.sdf")
    share_root = os.path.dirname(package_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gz_args = LaunchConfiguration("gz_args")
    entity_name = LaunchConfiguration("entity_name")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            entity_name,
            "-topic",
            "robot_description",
            "-allow_renaming",
            "true",
            "-z",
            "0.02",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/revolute_13_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/revolute_5_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/revolute_6_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/revolute_7_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/revolute_8_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/revolute_12_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
            ),
            DeclareLaunchArgument("gz_args", default_value=["-s -r ", world]),
            DeclareLaunchArgument("entity_name", default_value="complete_arm"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[share_root, ":", os.path.join("/root", "colcon_ws", "src")],
            ),
            gz_launch,
            rsp_node,
            spawn_entity,
            bridge,
        ]
    )
