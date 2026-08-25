from setuptools import find_packages, setup

package_name = 'conavgpt2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Vendored upstream Co-NavGPT2 library (mapping, frontier detection, agent)',
    license='TODO',
    # No executables. This package is a LIBRARY: it holds the vendored upstream
    # code and nothing else. The planner that uses it lives in search_baselines.
    entry_points={'console_scripts': []},
)
