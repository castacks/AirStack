from glob import glob
import os

from setuptools import find_packages, setup


package_name = "mononav_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AirLab",
    maintainer_email="airlab@andrew.cmu.edu",
    description="AirStack transport and trajectory adapter for external vision planners.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mononav_bridge_node = mononav_bridge.bridge_node:main",
        ],
    },
)
