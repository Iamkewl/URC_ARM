import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build MoveIt configs once and reuse the same robot_description everywhere
    moveit_config = (
        MoveItConfigsBuilder("rarm_5", package_name="my_robot_moveit_config")
        .robot_description(file_path="config/rarm_5.urdf.xacro")
        .to_moveit_configs()
    )

    # RViz config file
    rviz_config_file = os.path.join(
        moveit_config.package_path, "config", "moveit.rviz"
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    # Joint State Publisher (provides states for interactive planning)
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )

    # RViz2 with matching MoveIt parameters
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([
        robot_state_pub,
        joint_state_pub,
        move_group_node,
        rviz_node
    ])
