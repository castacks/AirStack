from setuptools import setup

package_name = 'airsim_depth_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.xml']),
        ('share/' + package_name + '/config', ['config/bridge.yaml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'bridge_node = airsim_depth_bridge.bridge_node:main',
        ],
    },
)
