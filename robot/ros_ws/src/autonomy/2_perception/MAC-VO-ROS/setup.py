import os
from setuptools import find_packages, setup

package_name = "macvo2"


def package_files(directory):
    paths = []
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join("..", path, filename))
    return paths


extra_files = package_files("MACVO_ROS2/src") + package_files("MACVO_ROS2/config")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # (
        #     "share/" + package_name + "/model",
        #     ["/root/model_weights/MACVO_FrontendCov.pth", "/root/model_weights/MACVO_posenet.pkl"],
        # ),
        ("share/" + package_name, ["launch/macvo.launch.xml"]),
        (
            "share/" + package_name + "/config",
            ["config/interface_config.yaml", "config/model_config.yaml"],
        ),
    ],
    package_data={package_name: extra_files},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yutian Chen",
    maintainer_email="yutianch@andrew.cmu.edu",
    description="ROS2 node wrapper for the MAC-VO",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "macvo2 = macvo2.macvo:main",
        ],
    },
)
