from setuptools import setup, find_namespace_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    name="skills_vg",
    packages=find_namespace_packages(where="utils"), 
    package_dir={"":"utils"},    
)

setup(**d)

