from setuptools import setup

package_name = 'global_plan_navigate_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/global_plan_navigate_bridge.launch.xml']),
        ('share/' + package_name + '/config', ['config/global_plan_navigate_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Jong',
    maintainer_email='ajong@andrew.cmu.edu',
    description='global_plan topic -> NavigateTask goal adapter for task-executor local planners',
    license='BSD-3-Clause-Clear',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'global_plan_navigate_bridge_node = global_plan_navigate_bridge.bridge_node:main',
        ],
    },
)
