"""Isaac Sim extension install metadata for the OptiTrack NatNet emulator."""

import os

from setuptools import find_packages, setup

EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))

setup(
    name="optitrack-natnet-emulator",
    version="0.1.0",
    description="NatNet UDP server emulator for Isaac Sim and natnet_ros2 integration",
    license="MIT",
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=[],
    packages=find_packages(where="."),
    package_dir={"": "."},
    zip_safe=False,
)
