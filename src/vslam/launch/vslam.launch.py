from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    vslam_pkg_prefix = get_package_share_directory('vslam')
    vslam_param_file = os.path.join(
        vslam_pkg_prefix, 'config/vslam_params.yaml')

    vslam_param = DeclareLaunchArgument(
        'vslam_param_file',
        default_value=vslam_param_file,
        description='Path to config file for vslam',
    )
    vslam = Node(
        package='vslam',
        executable='vslam_node',
        parameters=[LaunchConfiguration('vslam_param_file')],
        output='screen',
    )

    return LaunchDescription([
        vslam_param,
        vslam,
    ])
