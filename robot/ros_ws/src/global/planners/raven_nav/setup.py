from setuptools import find_packages, setup

package_name = 'raven_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/raven_nav.launch.xml']),
        ('share/' + package_name + '/config', ['config/raven_nav.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Navigation planner consuming RayFronts mapper outputs',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raven_nav_node = raven_nav.raven_nav_node:main',
        ],
    },
)
