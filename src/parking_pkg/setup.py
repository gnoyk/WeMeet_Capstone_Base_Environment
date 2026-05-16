from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parking_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/parking_pkg/launch', ['launch/valet.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='limo',
    maintainer_email='limo@todo.todo',
    description='Valet parking package for LIMO robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'limo_prepare_node = parking_pkg.limo_prepare_node:main',
            'limo_parking_node = parking_pkg.limo_parking_node:main',
        ],
    },
)
