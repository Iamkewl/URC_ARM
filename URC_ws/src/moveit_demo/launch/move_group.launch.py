from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rarm_5", package_name="moveit_demo")
        .robot_description(file_path="config/rarm_5.urdf.xacro")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # This ensures we don't pass complex objects that crash the launch system
    params = moveit_config.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([run_move_group_node])