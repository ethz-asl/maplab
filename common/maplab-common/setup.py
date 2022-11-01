# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD!

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml.
setup_args = generate_distutils_setup(
    packages=['maplab_common'],
    package_dir={'': 'python'},
)

setup(**setup_args)
