from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ros2_laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        output='screen',
    )

    return LaunchDescription(
        [
            ros2_laser_scan_matcher,
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
            ),
        ]
    )
