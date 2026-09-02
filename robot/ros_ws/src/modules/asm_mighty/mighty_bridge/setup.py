import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mighty_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('README.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Jong',
    maintainer_email='ajong@andrew.cmu.edu',
    description='MIGHTY to AirStack NavigateTask/trajectory_controller bridge',
    license='BSD-3-Clause-Clear',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mighty_bridge_node = mighty_bridge.bridge_node:main',
            # Native MIGHTY -> PX4 setpoint path ("Option A"), launched only
            # when MIGHTY_NATIVE_SETPOINTS is on. See README.md.
            'mighty_native_setpoint_node = '
            'mighty_bridge.native_setpoint_node:main',
        ],
    },
)
