import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'biguasim_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttszin',
    maintainer_email='matheuskrskr@gmail.com',
    description='ROS2 bridge for the BiguaSim T2 hybrid aerial/aquatic transition',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'biguasim_bridge = biguasim_bridge.bridge_node:main',
        ],
    },
)
