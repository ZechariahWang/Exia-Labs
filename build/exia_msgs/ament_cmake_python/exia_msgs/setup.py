from setuptools import find_packages
from setuptools import setup

setup(
    name='exia_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('exia_msgs', 'exia_msgs.*')),
)
