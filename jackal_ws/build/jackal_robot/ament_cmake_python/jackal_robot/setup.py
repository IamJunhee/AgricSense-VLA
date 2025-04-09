from setuptools import find_packages
from setuptools import setup

setup(
    name='jackal_robot',
    version='1.0.1',
    packages=find_packages(
        include=('jackal_robot', 'jackal_robot.*')),
)
