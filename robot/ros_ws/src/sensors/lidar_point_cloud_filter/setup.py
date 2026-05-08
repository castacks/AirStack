from setuptools import setup

package_name = 'lidar_point_cloud_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lidar_point_cloud_filter.launch.xml']),
        ('share/' + package_name + '/config', ['config/lidar_point_cloud_filter.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='AirLab CMU',
    maintainer_email='ajong@andrew.cmu.edu',
    description='Near-range sphere filter for lidar PointCloud2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_point_cloud_filter_node = lidar_point_cloud_filter.lidar_point_cloud_filter_node:main',
        ],
    },
)
