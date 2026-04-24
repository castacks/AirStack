import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'action_relay'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirLab CMU',
    maintainer_email='airlab@cmu.edu',
    description='Relay ROS 2 actions across DDS domains',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_relay_node = action_relay.relay_node:main',
        ],
    },
)
