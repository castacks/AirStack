from setuptools import setup

package_name = 'mocap_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.policy'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mocap_planner.launch.xml']),
        ('share/' + package_name + '/config', ['config/mocap_planner.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirLab CMU',
    maintainer_email='kmcfarla@andrew.cmu.edu',
    description='Simple mocap-based local planner with pluggable policy interface',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mocap_planner_node = mocap_planner.node:main',
        ],
    },
)
