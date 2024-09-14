from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['trackbot'],  # Replace with your package's name if different
    package_dir={'': 'src'},
)

setup(**setup_args)
