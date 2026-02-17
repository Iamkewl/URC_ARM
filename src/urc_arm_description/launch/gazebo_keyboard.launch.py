from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    use_keyboard_teleop = LaunchConfiguration("use_keyboard_teleop")
    teleop_mode = LaunchConfiguration("teleop_mode")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"gui": gui}.items(),
    )

    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare("urc_arm_description"), "urdf", "rarm_5.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content, "use_sim_time": use_sim_time}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "rarm_5", "-topic", "robot_description"],
    )

    keyboard_teleop = Node(
        package="urc_arm_description",
        executable="keyboard_arm_teleop.py",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'",
                use_keyboard_teleop,
                "'.lower() == 'true' and '",
                teleop_mode,
                "' == 'joint'",
            ])
        ),
    )

    cartesian_teleop = Node(
        package="urc_arm_description",
        executable="keyboard_cartesian_teleop.py",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'",
                use_keyboard_teleop,
                "'.lower() == 'true' and '",
                teleop_mode,
                "' == 'cartesian'",
            ])
        ),
    )

    start_teleop_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[keyboard_teleop, cartesian_teleop],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_keyboard_teleop", default_value="true"),
            DeclareLaunchArgument("teleop_mode", default_value="joint"),
            gazebo_launch,
            robot_state_publisher,
            spawn_entity,
            start_teleop_after_spawn,
        ]
    )
