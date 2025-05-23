from setuptools import find_packages, setup

package_name = 'ring_buffer_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan M',
    maintainer_email='martnat8@oregonstate.edu',
    description='A lifecycle compliant node that creates a buffer of images from a subscription topic and ' \
    'on service call saves that to disk',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ring_buffer_recorder = ring_buffer_recorder.ring_buffer_recorder:main',
        ],
    },
)
