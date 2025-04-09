from setuptools import find_packages
from setuptools import setup

setup(
    name='jackal_msgs',
    version='1.0.4',
    packages=find_packages(
        include=('jackal_msgs', 'jackal_msgs.*')),
)
