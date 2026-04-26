from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="path_tracker",
                executable="path_tracker_node",
                name="path_tracker",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("path_tracker"), "config", "path_tracker.yaml"]
                    )
                ],
            )
        ]
    )
