from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rqt_fixed_trajectory_selector'],
    package_dir={'': 'src'},
)

setup(**d)
