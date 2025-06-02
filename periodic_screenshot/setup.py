from setuptools import find_packages, setup

package_name = 'periodic_screenshot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='Nathan M',
    maintainer_email='martnat8@oregonstate.edu',
    description='This node subscribes to a sensor_msgs/Image topic and saves an image to disk at a parameterized rate',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'periodic_screenshot = periodic_screenshot.periodic_screenshot:main',
        ],
    },
)
