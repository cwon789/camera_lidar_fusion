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
    default_rviz = os.path.join(package_share, 'rviz', 'colorized_pointcloud.rviz')

    calib_path = LaunchConfiguration('calib_path')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('calib_path', default_value=default_calib),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lidar_frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera'),
        DeclareLaunchArgument('max_points', default_value='0'),
        DeclareLaunchArgument('max_image_age_sec', default_value='0.0'),
        DeclareLaunchArgument('keep_uncolored', default_value='true'),
        DeclareLaunchArgument('use_inverse_transform', default_value='true'),

        Node(
            package='lidar_camera_colorizer',
            executable='calib_tf_publisher',
            name='calib_tf_publisher',
            output='screen',
            parameters=[{
                'calib_path': calib_path,
                'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'use_inverse_transform': LaunchConfiguration('use_inverse_transform'),
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
                'max_points': LaunchConfiguration('max_points'),
                'max_image_age_sec': LaunchConfiguration('max_image_age_sec'),
                'keep_uncolored': LaunchConfiguration('keep_uncolored'),
                'use_inverse_transform': LaunchConfiguration('use_inverse_transform'),
            }],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
