from glob import glob

from setuptools import find_packages, setup

package_name = 'conavgpt2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/conavgpt2.launch.xml']),
        # Every overlay, not just the base: the launch file takes a
        # method_params_file and a scene_params_file, and a mission names them
        # by their INSTALLED path.
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='ROS 2 wrapper around upstream Co-NavGPT2 multi-robot VLM frontier assignment',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conavgpt2_node = conavgpt2.conavgpt2_node:main',
        ],
    },
)
