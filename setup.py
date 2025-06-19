from setuptools import find_packages, setup

package_name = 'lidar_human_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'lidar_human_detect.pcd_to_csv_node',
        'lidar_human_detect.human_detect_node',
        'lidar_human_detect.dist_calc_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [])  # Add launch files later here
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pandas',
        'open3d',
        'scikit-learn',
        'scipy',
        'rclpy'
    ],
    zip_safe=True,
    maintainer='eshan',
    maintainer_email='eshan@todo.todo',
    description='Human detection and analysis in LiDAR point clouds using ROS 2 and Open3D',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pcd_to_csv = lidar_human_detect.pcd_to_csv_node:main',
            'detect_human = lidar_human_detect.human_detect_node:main',
            'dist_calc = lidar_human_detect.dist_calc_node:main',
        ],
    },
)
