import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("rove_description"),
        "urdf",
        "rove_standard.urdf"
    )

    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox',
            arguments=['--x', '0', '--y', '0', '--z', '0.1',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'livox_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='core_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'Core',
                       '--child-frame-id', 'base_link'],
        ),
        Node(
            package="rviz2",
            namespace="",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d" + os.path.join(
                    get_package_share_directory("rove_description"),
                    "rviz",
                    "conf.rviz",
                )
            ],
        ),
    ])