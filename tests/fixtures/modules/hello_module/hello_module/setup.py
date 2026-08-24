from setuptools import setup

package_name = "hello_module"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AirStack Test Fixture",
    maintainer_email="test@example.com",
    description="Minimal hello-world fixture package for module manifest contract tests.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "hello_node = hello_module.hello_node:main",
        ],
    },
)
