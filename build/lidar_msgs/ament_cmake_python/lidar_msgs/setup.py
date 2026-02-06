from setuptools import find_packages
from setuptools import setup

setup(
    name='lidar_msgs',
    version='0.2.0',
    packages=find_packages(
        include=('lidar_msgs', 'lidar_msgs.*')),
)
