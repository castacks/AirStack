from setuptools import setup

package_name = 'rqt_behavior_tree'

setup(
    name=package_name,
    version='1.0.2',
    packages=[package_name, package_name + '/xdot'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/py_console_widget.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dorian Scholz',
    maintainer='Dirk Thomas, Dorian Scholz, Michael Lautman',
    maintainer_email='dthomas@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rqt_behavior_tree is a Python GUI plugin providing an interactive Python console.'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'rqt_behavior_tree = ' + package_name + '.main:main',
        ],
    },
)
