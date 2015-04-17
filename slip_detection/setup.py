#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['slip_detection'],
    package_dir={'slip_detection': 'src'}
)

setup(**d)