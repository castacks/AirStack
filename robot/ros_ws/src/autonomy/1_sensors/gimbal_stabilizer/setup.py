from setuptools import find_packages, setup

package_name = 'gimbal_stabilizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Package for gimbal stabilization using roll, pitch, and yaw from odometry data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gimbal_stabilizer_node = gimbal_stabilizer.gimbal_stabilizer_node:main'
        ],
    },
)