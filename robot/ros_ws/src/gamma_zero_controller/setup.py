from glob import glob
from setuptools import setup
setup(name="gamma_zero_controller", version="0.1.0", packages=["gamma_zero_controller"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/gamma_zero_controller"]),
                  ("share/gamma_zero_controller", ["package.xml"]),
                  ("share/gamma_zero_controller/launch", glob("launch/*.py")),
                  ("share/gamma_zero_controller/config", glob("config/*"))],
      install_requires=["setuptools", "numpy"], maintainer="Yutong Wang",
      maintainer_email="yutongw3@andrew.cmu.edu", description="BV Gamma-Zero CTBR backend", license="Proprietary",
      entry_points={"console_scripts": ["controller = gamma_zero_controller.node:main", "policy_worker = gamma_zero_controller.policy_worker:main"]})
