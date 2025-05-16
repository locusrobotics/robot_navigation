#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['nav_2d_utils'],
    package_dir={'': 'src'}
)

setup(**package_info)
