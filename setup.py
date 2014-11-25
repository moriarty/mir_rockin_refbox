#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mir_rockin_refbox'],
   package_dir={'mir_rockin_refbox': 'ros/src'}
)

distutils.core.setup(**d)
