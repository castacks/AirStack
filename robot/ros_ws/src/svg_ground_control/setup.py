import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'svg_ground_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yikuan Fang',
    maintainer_email='yikuanfang@gmail.com',
    description='Multi-drone mocap ground controller with CBF safety filter placeholder',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_commander = svg_ground_control.swarm_commander:main',
            'diffaero_commander = svg_ground_control.diffaero_commander:main',
            'diffaero_velocity_commander = svg_ground_control.diffaero_velocity_commander:main',
            'mocap_bridge = svg_ground_control.mocap_bridge:main',
            'keyboard_teleop = svg_ground_control.keyboard_teleop:main',
        ],
    },
)
