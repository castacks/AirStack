from setuptools import find_packages, setup

package_name = 'coordination_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gossip.launch.xml']),
        ('share/' + package_name + '/config', ['config/gossip_dds_router.yaml']),
        ('lib/' + package_name, ['scripts/gossip_node', 'scripts/peer_registry_monitor.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirLab',
    maintainer_email='airlab@andrew.cmu.edu',
    description='Gossip-protocol multi-agent coordination layer for AirStack',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gossip_node = coordination_bringup.gossip_node:main',
        ],
    },
)
