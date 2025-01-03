import os
from setuptools import find_packages, setup

package_name = 'macvo'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

extra_files = package_files('macvo/src')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/macvo.launch.xml']),
        ('share/' + package_name + '/config', ['config/interface_config.yaml','config/model_config.yaml']),
    ],
    package_data={package_name: extra_files},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yutian',
    maintainer_email='markchenyutian@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'macvo = macvo.macvo:main',
        ],
    },
)
