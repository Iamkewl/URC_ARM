import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('urc_arm_description')
    
    # 1. Process the URDF (Xacro)
    xacro_file = os.path.join(pkg_path, 'urdf', 'urc_arm.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    # 2. Nodes
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),
        # Joint State Publisher GUI (The sliders you saw earlier)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'view_arm.rviz')]
        ),
        # NEW: Static Transform for the Camera
        # Adjust '0 0 0.05' to the real distance from your wrist to the camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=['0', '0', '0.05', '0', '0', '0', 'link_5', 'camera_link']
        ),
        # NEW: Camera Optical Frame (Required for OpenCV/Image math)
        # This rotates the frame so Z points "into" the image
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_broadcaster',
            arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link', 'camera_color_optical_frame']
        )
    ])