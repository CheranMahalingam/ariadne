from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    cuda_vslam_ros_pkg_prefix = get_package_share_directory(
        'cuda_vslam_ros')
    vslam_param_file = os.path.join(
        cuda_vslam_ros_pkg_prefix, 'config/vslam_params.yaml')
    foxglove_param_file = os.path.join(
        cuda_vslam_ros_pkg_prefix, 'config/foxglove_params.yaml')

    vslam_param = DeclareLaunchArgument(
        'vslam_param_file',
        default_value=vslam_param_file,
        description='Path to config file for vslam',
    )
    vslam = Node(
        package='cuda_vslam_ros',
        executable='vslam_node',
        parameters=[LaunchConfiguration('vslam_param_file')],
        output='screen',
    )

    foxglove_param = DeclareLaunchArgument(
        'foxglove_param_file',
        default_value=foxglove_param_file,
        description='Path to config file for foxglove bridge',
    )
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[LaunchConfiguration('foxglove_param_file')],
        output='screen',
    )

    return LaunchDescription([
        vslam_param,
        vslam,
        foxglove_param,
        foxglove_bridge,
    ])
