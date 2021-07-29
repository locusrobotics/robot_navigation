#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['rqt_dwb_plugin'],
    package_dir={'': 'src'}
)

setup(**package_info)
