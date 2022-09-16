from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['grasp_generator'],
    package_dir={'': 'scripts'}
)
setup(**d)
