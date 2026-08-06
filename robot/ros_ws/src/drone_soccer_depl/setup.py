from setuptools import find_packages, setup

package_name = 'drone_soccer_depl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'model_1000_before.pt',
            'model_1000_after.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bavin',
    maintainer_email='bavinsaravanan24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'inference_soccer_pos_sp = '
            'drone_soccer_depl.inference_soccer_pos_sp:main',
            'pos_sp_soccer = drone_soccer_depl.pos_sp_soccer:main',
        ],
    },
)
