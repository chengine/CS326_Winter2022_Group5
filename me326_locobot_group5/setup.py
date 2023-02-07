from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['me326_locobot_group5'],
    scripts=['scripts/locobot_example_motion.py'],
    package_dir={'': 'src'}
)

setup(**d)

