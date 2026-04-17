from setuptools import find_packages, setup

package_name = 'semantic_search_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/semantic_search_task.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='SemanticSearchTask action server using rayfronts + raven_nav',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_search_task = semantic_search_task.node:main',
        ],
    },
)
