from setuptools import setup
from glob import glob

package_name = 'rqt_behavior_tree_command'

setup(
    name=package_name,
    version='1.0.2',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/py_console_widget.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/config/', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='',
    maintainer='',
    maintainer_email='',
    keywords=['ROS'],
    classifiers=[
        '',
        '',
        '',
        '',
    ],
    description=(
        'rqt_behavior_tree_command'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'rqt_behavior_tree_command = ' + package_name + '.main:main',
        ],
    },
)
