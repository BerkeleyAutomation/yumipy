"""
Setup of YuMiPy  codebase
Author: Jacky Liang
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['yumipy'],
    package_dir={'': ''}, )
setup(**setup_args)
