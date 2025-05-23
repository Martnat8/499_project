
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'package_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # These lines make sure the launch files are installed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teft',
    maintainer_email='martnat8@oregonstate.edu',
    description='Package that launches all needed nodes for ROB 499 Final Project',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
