from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('lidar_camera_colorizer')
    default_calib = '/home/jay/catkin_depth/src/livox_preprocessed/calib.json'
    default_data_dir = '/home/jay/catkin_depth/src/livox_preprocessed'
    default_rviz = os.path.join(package_share, 'rviz', 'colorized_pointcloud.rviz')

    calib_path = LaunchConfiguration('calib_path')
    data_dir = LaunchConfiguration('data_dir')
    sample_name = LaunchConfiguration('sample_name')
    point_limit = LaunchConfiguration('point_limit')

    return LaunchDescription([
        DeclareLaunchArgument('calib_path', default_value=default_calib),
        DeclareLaunchArgument('data_dir', default_value=default_data_dir),
        DeclareLaunchArgument('sample_name', default_value='rosbag2_2023_03_09-13_42_46'),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('point_limit', default_value='0'),
        DeclareLaunchArgument('max_image_age_sec', default_value='0.0'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='lidar_camera_colorizer',
            executable='calib_tf_publisher',
            name='calib_tf_publisher',
            output='screen',
            parameters=[{
                'calib_path': calib_path,
                'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'use_inverse_transform': True,
            }],
        ),

        Node(
            package='lidar_camera_colorizer',
            executable='demo_publisher',
            name='demo_publisher',
            output='screen',
            parameters=[{
                'calib_path': calib_path,
                'data_dir': data_dir,
                'sample_name': sample_name,
                'point_limit': point_limit,
                'frame_id': LaunchConfiguration('lidar_frame_id'),
                'publish_rate_hz': 2.0,
            }],
        ),

        Node(
            package='lidar_camera_colorizer',
            executable='colorize_node',
            name='colorize_node',
            output='screen',
            parameters=[{
                'calib_path': calib_path,
                'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'max_points': point_limit,
                'keep_uncolored': True,
                'use_inverse_transform': True,
                'max_image_age_sec': LaunchConfiguration('max_image_age_sec'),
            }],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
