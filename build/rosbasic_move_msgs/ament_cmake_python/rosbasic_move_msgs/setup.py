from setuptools import find_packages
from setuptools import setup

setup(
    name='rosbasic_move_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('rosbasic_move_msgs', 'rosbasic_move_msgs.*')),
)
