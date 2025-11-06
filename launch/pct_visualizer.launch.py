#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_dir = get_package_share_directory('pct_planner')
    pkg_prefix = get_package_prefix('pct_planner')

    # Get path to default parameter file
    default_params_file = os.path.join(pkg_dir, 'config', 'pct_planner_params.yaml')

    planner_lib_dir = os.path.join(pkg_prefix, '..', '..', 'src', 'route_planner', 'PCT_planner', 'pct_planner', 'planner', 'lib')
    gtsam_lib_dir = os.path.join(planner_lib_dir, '3rdparty', 'gtsam-4.1.1', 'install', 'lib')
    smoothing_lib_dir = os.path.join(planner_lib_dir, 'build', 'src', 'common', 'smoothing')

    existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f"{gtsam_lib_dir}:{smoothing_lib_dir}:{existing_ld_path}"

    existing_python_path = os.environ.get('PYTHONPATH', '')
    new_python_path = f"{planner_lib_dir}:{existing_python_path}"

    # Declare launch arguments
    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='',
        description='Path to PCD file to visualize'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to PCT planner parameters YAML file'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Rate (Hz) to publish visualization (default: 1.0)'
    )

    publish_layers_arg = DeclareLaunchArgument(
        'publish_layers',
        default_value='false',
        description='Publish individual tomogram layers (default: false)'
    )

    pct_visualizer_node = Node(
        package='pct_planner',
        executable='pct_planner_visualizer.py',
        name='pct_planner_visualizer',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'pcd_file': LaunchConfiguration('pcd_file'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_layers': LaunchConfiguration('publish_layers'),
            }
        ],
        additional_env={'LD_LIBRARY_PATH': new_ld_path, 'PYTHONPATH': new_python_path}
    )

    return LaunchDescription([
        pcd_file_arg,
        params_file_arg,
        publish_rate_arg,
        publish_layers_arg,
        LogInfo(msg=['Starting PCT Planner Visualizer with PCD file: ', LaunchConfiguration('pcd_file')]),
        pct_visualizer_node,
    ])
