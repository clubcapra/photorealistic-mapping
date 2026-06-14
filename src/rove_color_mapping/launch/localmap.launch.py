from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("local_size",   default_value="200",
                              description="Output crop size in cells (NxN)"),
        DeclareLaunchArgument("http_port",    default_value="8765",
                              description="HTTP port for the debug server"),
        DeclareLaunchArgument("map_topic",    default_value="/grid_prob_map",
                              description="rtabmap occupancy grid topic"),
        DeclareLaunchArgument("robot_frame",  default_value="Core",
                              description="Robot base TF frame"),
        DeclareLaunchArgument("map_frame",    default_value="new_map",
                              description="Map TF frame"),

        Node(
            package="rove_color_mapping",
            executable="local_map_node",
            name="local_map_server",
            output="screen",
            parameters=[{
                "local_size":   LaunchConfiguration("local_size"),
                "http_port":    LaunchConfiguration("http_port"),
                "map_topic":    LaunchConfiguration("map_topic"),
                "robot_frame":  LaunchConfiguration("robot_frame"),
                "map_frame":    LaunchConfiguration("map_frame"),
            }],
        ),
    ])