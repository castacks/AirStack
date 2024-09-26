import os
from glob import glob
from setuptools import setup

package_name = "tf_relay"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrew Jong",
    maintainer_email="ajong@andrew.cmu.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "relay = tf_relay.main:main",
            "tf_relay = tf_relay.tf_relay:main",
            "tf_static_relay = tf_relay.tf_static_relay:main",
            ],
    },
)
