from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
import signal
import sys

def generate_launch_description():
    def signal_handler(sig, frame):
        print("\nCtrl+C shutdown")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    return LaunchDescription([
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                ]
            )
        ),

        Node(
            package='lidar_human_detect',
            executable='pcd_to_csv',
            name='pcd_to_csv',
            output='screen',
            parameters=[{'pcd_file': '/mnt/e/IITH/LiDAR/fastlio_ws/src/unilidar_fastlio_ros2/PCD/scans.pcd'},
                        {'csv_file': '/mnt/e/IITH/LiDAR/lidar_ws/scans.csv'}]
        ),

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='lidar_human_detect',
                    executable='detect_human',
                    name='detect_human',
                    output='screen',
                    parameters=[{'csv_file': '/mnt/e/IITH/LiDAR/lidar_ws/scans.csv'},
                                {'output_file': '/mnt/e/IITH/LiDAR/lidar_ws/human_cluster.csv'}]
                )
            ]
        ),

        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='lidar_human_detect',
                    executable='dist_calc',
                    name='dist_calc',
                    output='screen',
                    parameters=[{'csv_file_path': '/mnt/e/IITH/LiDAR/lidar_ws/human_cluster.csv'}]
                )
            ]
        )
    ])
