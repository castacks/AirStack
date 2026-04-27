from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gcs_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirLab CMU',
    maintainer_email='airlab@cmu.edu',
    description='GCS visualization node for drone mesh markers in Foxglove',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foxglove_visualizer_node = gcs_visualizer.foxglove_visualizer_node:main',
            'payload_visualizer_node = gcs_visualizer.payload_visualizer_node:main',
            'waypoint_collector_node = gcs_visualizer.waypoint_collector_node:main',
            'polygon_collector_node = gcs_visualizer.polygon_collector_node:main',
        ],
    },
)
