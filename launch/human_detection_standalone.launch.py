#!/usr/bin/env python3
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    human_detect_pkg = get_package_share_directory('lidar_human_detect')
    rviz_config_file = os.path.join(human_detect_pkg, 'rviz', 'human_detection.rviz')

    detection_x_min = LaunchConfiguration('detection_x_min')
    detection_x_max = LaunchConfiguration('detection_x_max')
    detection_y_min = LaunchConfiguration('detection_y_min')
    detection_y_max = LaunchConfiguration('detection_y_max')
    detection_z_min = LaunchConfiguration('detection_z_min')
    detection_z_max = LaunchConfiguration('detection_z_max')
    voxel_size = LaunchConfiguration('voxel_size')

    declare_detection_x_min_cmd = DeclareLaunchArgument(
        'detection_x_min', default_value='0.0',
        description='Minimum X coordinate for detection region (meters)'
    )
    declare_detection_x_max_cmd = DeclareLaunchArgument(
        'detection_x_max', default_value='3.0',
        description='Maximum X coordinate for detection region (meters)'
    )
    declare_detection_y_min_cmd = DeclareLaunchArgument(
        'detection_y_min', default_value='-1.0',
        description='Minimum Y coordinate for detection region (meters)'
    )
    declare_detection_y_max_cmd = DeclareLaunchArgument(
        'detection_y_max', default_value='1.0',
        description='Maximum Y coordinate for detection region (meters)'
    )
    declare_detection_z_min_cmd = DeclareLaunchArgument(
        'detection_z_min', default_value='0.0',
        description='Minimum Z coordinate for detection region (meters)'
    )
    declare_detection_z_max_cmd = DeclareLaunchArgument(
        'detection_z_max', default_value='5.0',
        description='Maximum Z coordinate for detection region (meters)'    )
    declare_voxel_size_cmd = DeclareLaunchArgument(
        'voxel_size', default_value='0.05',
        description='Voxel size for point cloud processing (meters)'    )

    human_detect_node = Node(
        package='lidar_human_detect',
        executable='realtime_human_detect',
        name='realtime_human_detect',
        parameters=[
            {'x_min': detection_x_min},
            {'x_max': detection_x_max},
            {'y_min': detection_y_min},
            {'y_max': detection_y_max},
            {'z_min': detection_z_min},
            {'z_max': detection_z_max},
            {'voxel_size': voxel_size}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',        
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_detection_x_min_cmd)
    ld.add_action(declare_detection_x_max_cmd)
    ld.add_action(declare_detection_y_min_cmd)
    ld.add_action(declare_detection_y_max_cmd)
    ld.add_action(declare_detection_z_min_cmd)
    ld.add_action(declare_detection_z_max_cmd)
    ld.add_action(declare_voxel_size_cmd)
    
    ld.add_action(human_detect_node)
    ld.add_action(rviz_node)
    
    return ld