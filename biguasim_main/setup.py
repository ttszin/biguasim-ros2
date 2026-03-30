from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'biguasim_main'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),   
        (os.path.join('share',package_name,'config'),glob('config/*.json')),  
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),  
        (os.path.join('share',package_name,'scripts'),glob('scripts/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgmateus',
    maintainer_email='eng.mgmateus@gmail.com',
    description='Ros Interface for Hybrid Aerial-Underwater',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'biguasim_node = biguasim_main.biguasim_node:main'
        ],
    },
)
