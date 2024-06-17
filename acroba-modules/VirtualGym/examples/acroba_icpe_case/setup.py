from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['acroba_icpe_case'],
    package_dir={'': 'src'}
)

setup(**d)
