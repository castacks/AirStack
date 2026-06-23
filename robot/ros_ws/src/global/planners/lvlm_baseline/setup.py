from setuptools import find_packages, setup

package_name = 'lvlm_baseline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lvlm_baseline.launch.xml']),
        ('share/' + package_name + '/config', ['config/lvlm_baseline.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='FPV + LVLM (InternVL3) navigation baseline driving the drone via NavigateTask',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lvlm_baseline_node = lvlm_baseline.lvlm_baseline_node:main',
        ],
    },
)
