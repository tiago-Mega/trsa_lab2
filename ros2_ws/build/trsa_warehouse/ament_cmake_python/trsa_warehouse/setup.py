from setuptools import find_packages
from setuptools import setup

setup(
    name='trsa_warehouse',
    version='1.2.0',
    packages=find_packages(
        include=('trsa_warehouse', 'trsa_warehouse.*')),
)
