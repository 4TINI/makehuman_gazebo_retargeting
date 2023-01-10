from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['scripts/actor', 'scripts/bvh_skeleton'],
    # packages=['makehuman_gazebo_retargeting'],
    # package_dir={'': 'scripts'}
)

setup(**d)