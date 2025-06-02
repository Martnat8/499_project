from setuptools import find_packages, setup

package_name = 'lifecycle_coordinator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Nathan M',
    maintainer_email='martnat8@oregonstate.edu',
    description='A coordinator for lifecycle enabled nodes. Can control multiple nodes' \
    'at a time through a parameterized list. Used through service calls',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_coordinator = lifecycle_coordinator.coordinator:main',
        ],
    },
)
