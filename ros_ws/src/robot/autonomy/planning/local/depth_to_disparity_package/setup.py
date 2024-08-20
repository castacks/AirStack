from setuptools import setup

package_name = "depth_to_disparity_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "opencv-python", "cv_bridge"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="A package that converts depth images to disparity images",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_to_disparity = depth_to_disparity_package.depth_to_disparity:main",
        ],
    },
)
