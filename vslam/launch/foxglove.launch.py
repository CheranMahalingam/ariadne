from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    cuda_vslam_ros_pkg_prefix = get_package_share_directory(
        'cuda_vslam_ros')
    foxglove_param_file = os.path.join(
        cuda_vslam_ros_pkg_prefix, 'config/foxglove_params.yaml')

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
        foxglove_param,
        foxglove_bridge,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("cuda_vslam_ros"), "/launch", "/vslam.launch.py"])
            ),
    ])
