from glob import glob

from setuptools import find_packages, setup

package_name = 'raven_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/raven_nav.launch.xml']),
        ('share/' + package_name + '/config', ['config/raven_nav.yaml']),
        # Ground-truth scene annotations, mirrored from gcs_visualizer/annotations
        # so the metrics scripts can score detections against ground truth
        # in-container (the robot workspace has no GCS source). Keep in sync if
        # the canonical GCS annotations change.
        ('share/' + package_name + '/annotations', glob('annotations/*.json')),
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
            'view_ray_tables = raven_nav.view_ray_tables:main',
            'compile_results = raven_nav.compile_results:main',
            'compare_to_groundtruth = raven_nav.compare_to_groundtruth:main',
        ],
    },
)
