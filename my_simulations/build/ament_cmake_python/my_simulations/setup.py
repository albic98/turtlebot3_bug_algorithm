from setuptools import find_packages
from setuptools import setup

setup(
    name='my_simulations',
    version='0.0.0',
    packages=find_packages(
        include=('my_simulations', 'my_simulations.*')),
)
