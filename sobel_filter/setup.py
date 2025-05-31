from setuptools import find_packages, setup

package_name = 'sobel_filter'

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
    description='This package subscribes to a image topic and runs a sobel filter then republishes, is a lifecycle node',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sobel_filter = sobel_filter.sobel_filter:main'
        ],
    },
)
