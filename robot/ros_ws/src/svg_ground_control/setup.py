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
        (os.path.join('share', package_name, 'config', 'drone_soccer'),
         glob('config/drone_soccer/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools', 'stable-baselines3==2.8.0', 'numpy'],
    zip_safe=True,
    maintainer='Yikuan Fang',
    maintainer_email='yikuanfang@gmail.com',
    description='Multi-drone mocap ground controller with CBF safety filter placeholder',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_commander = svg_ground_control.swarm_commander:main',
            'mocap_bridge = svg_ground_control.mocap_bridge:main',
            'plot_mocap_velocity = '
            'svg_ground_control.mocap_velocity_plotter:main',
            'keyboard_teleop = svg_ground_control.keyboard_teleop:main',
            'trajectory_commander = svg_ground_control.trajectory_commander:main',
            'policy_commander = svg_ground_control.policy_commander:main',
        ],
    },
)
