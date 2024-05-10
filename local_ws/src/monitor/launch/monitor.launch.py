import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():
    package_name = "monitor"
    map_file_name = "slam.yaml"

    pkg_path = os.path.join(get_package_share_directory(package_name))

    map_path = os.path.join(pkg_path, "map", map_file_name)

    return LaunchDescription(
        [
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
            # ),
            # Node(
            #     package='monitor',
            #     executable='rviz_odom',
            #     output='screen',
            # ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_path}]
            ),
            launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='map_lifecycle_manager',  # unique name
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': ['map_server']}]
            ),
        ]
    )
