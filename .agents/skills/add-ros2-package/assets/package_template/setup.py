from setuptools import setup
import os
from glob import glob

# TODO: Replace with your package name
package_name = 'your_package_name'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        # Install config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') + glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # TODO: Update
    maintainer_email='your.email@example.com',  # TODO: Update
    description='Brief description of your module',  # TODO: Update
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TODO: Add your node executables here
            # Format: 'executable_name = package_name.module_name:main'
            'your_node_name = your_package_name.your_node:main'
        ],
    },
)
