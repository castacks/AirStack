from setuptools import find_packages, setup

package_name = 'ros2tak_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mission-operator',
    maintainer_email='mission-operator@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2cot_agent = ros2tak_tools.ros2cot_agent:main',
            'cot2ros_agent = ros2tak_tools.cot2ros_agent:main',
            'cot2planner_agent = ros2tak_tools.cot2planner_agent:main',
            'ros2casevac_agent = ros2tak_tools.ros2casevac_agent:main',
            'chat2ros_agent = ros2tak_tools.chat2ros_agent:main',
        ],
    },
)
