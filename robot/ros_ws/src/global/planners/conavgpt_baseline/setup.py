from setuptools import find_packages, setup

package_name = 'conavgpt_baseline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/conavgpt_baseline.launch.xml']),
        ('share/' + package_name + '/config', ['config/conavgpt_baseline.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Co-NavGPT VLM-Assign baseline: team-level frontier assignment from one InternVL3 call per round',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conavgpt_assigner_node = conavgpt_baseline.conavgpt_assigner_node:main',
        ],
    },
)
