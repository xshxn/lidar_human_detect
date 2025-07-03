#!/usr/bin/env python3
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    fast_lio_pkg = get_package_share_directory('fast_lio')
    human_detect_pkg = get_package_share_directory('lidar_human_detect')
    
    default_config_path = os.path.join(fast_lio_pkg, 'config')
    default_rviz_config_path = os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')
    human_detect_rviz_config = os.path.join(human_detect_pkg, 'rviz', 'human_detection.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    human_detection_enable = LaunchConfiguration('human_detection_enable')
    detection_x_min = LaunchConfiguration('detection_x_min')
    detection_x_max = LaunchConfiguration('detection_x_max')
    detection_y_min = LaunchConfiguration('detection_y_min')
    detection_y_max = LaunchConfiguration('detection_y_max')
    detection_z_min = LaunchConfiguration('detection_z_min')
    detection_z_max = LaunchConfiguration('detection_z_max')
    voxel_size = LaunchConfiguration('voxel_size')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='unilidar_l2.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=human_detect_rviz_config,
        description='RViz config file path'
    )
    declare_human_detection_enable_cmd = DeclareLaunchArgument(
        'human_detection_enable', default_value='true',
        description='Enable real-time human detection'
    )
    declare_detection_x_min_cmd = DeclareLaunchArgument(
        'detection_x_min', default_value='0.0',
        description='Minimum X coordinate for detection region'
    )
    declare_detection_x_max_cmd = DeclareLaunchArgument(
        'detection_x_max', default_value='3.0',
        description='Maximum X coordinate for detection region'
    )
    declare_detection_y_min_cmd = DeclareLaunchArgument(
        'detection_y_min', default_value='-1.0',
        description='Minimum Y coordinate for detection region'
    )
    declare_detection_y_max_cmd = DeclareLaunchArgument(
        'detection_y_max', default_value='1.0',
        description='Maximum Y coordinate for detection region'
    )
    declare_detection_z_min_cmd = DeclareLaunchArgument(
        'detection_z_min', default_value='0.0',
        description='Minimum Z coordinate for detection region'
    )
    declare_detection_z_max_cmd = DeclareLaunchArgument(
        'detection_z_max', default_value='5.0',
        description='Maximum Z coordinate for detection region'
    )
    declare_voxel_size_cmd = DeclareLaunchArgument(
        'voxel_size', default_value='0.05',
        description='Voxel size for point cloud processing'
    )
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    odom_to_tf_node = Node(
        package='fast_lio',
        executable='odometry_to_tf',
        name='odometry_to_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    
    human_detect_node = Node(
        package='lidar_human_detect',
        executable='realtime_human_detect',
        name='realtime_human_detect',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'x_min': detection_x_min},
            {'x_max': detection_x_max},
            {'y_min': detection_y_min},
            {'y_max': detection_y_max},
            {'z_min': detection_z_min},
            {'z_max': detection_z_max},
            {'voxel_size': voxel_size}
        ],
        output='screen',
        condition=IfCondition(human_detection_enable)
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use),
        output='screen'
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_human_detection_enable_cmd)
    ld.add_action(declare_detection_x_min_cmd)
    ld.add_action(declare_detection_x_max_cmd)
    ld.add_action(declare_detection_y_min_cmd)
    ld.add_action(declare_detection_y_max_cmd)
    ld.add_action(declare_detection_z_min_cmd)
    ld.add_action(declare_detection_z_max_cmd)
    ld.add_action(declare_voxel_size_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(odom_to_tf_node)
    ld.add_action(human_detect_node)
    ld.add_action(rviz_node)
    
    return ld
