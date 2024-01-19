from setuptools import find_packages
from setuptools import setup

setup(
    name='log_gpis',
    version='0.0.1',
    packages=find_packages(
        include=('log_gpis', 'log_gpis.*')),
)
