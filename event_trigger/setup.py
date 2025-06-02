from setuptools import find_packages, setup

package_name = 'event_trigger'

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
    description='An Event Trigger node triggers the Rign Buffer Recorder node on a timer',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event_trigger = event_trigger.event_trigger::main'
        ],
    },
)
