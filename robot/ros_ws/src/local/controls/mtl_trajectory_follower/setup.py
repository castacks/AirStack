import os
from glob import glob

from setuptools import setup

package_name = "mtl_trajectory_follower"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Burhan",
    maintainer_email="bshirose@andrew.cmu.edu",
    description="Arc-length carrot pursuit and scheduled gimbal pointing for MTL search sorties.",
    license="BSD-3-Clause-Clear",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "mtl_trajectory_follower = mtl_trajectory_follower.follower_node:main",
        ],
    },
)
