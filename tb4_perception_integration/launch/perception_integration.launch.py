"""Launch file for the Camera–LiDAR fusion node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb4_perception_integration')

    config_file = os.path.join(pkg_dir, 'config', 'fusion_params.yaml')

    publish_debug_markers_arg = DeclareLaunchArgument(
        'publish_debug_markers', default_value='false',
        description='Enable debug MarkerArray on /fusion_debug_markers')

    fusion_node = Node(
        package='tb4_perception_integration',
        executable='camera_lidar_fusion_node',
        name='camera_lidar_fusion_node',
        output='screen',
        parameters=[config_file, {
            'publish_debug_markers': LaunchConfiguration('publish_debug_markers'),
        }],
    )

    return LaunchDescription([
        publish_debug_markers_arg,
        fusion_node,
    ])
