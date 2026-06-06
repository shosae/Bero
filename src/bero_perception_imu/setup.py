from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bero_perception_imu'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='shosae',
    maintainer_email='phone13324@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'floor_estimator = bero_perception_imu.floor_estimator:main',
            'floor_estimator_viz = bero_perception_imu.floor_estimator_viz:main',
            'az_offset_calibrator = bero_perception_imu.az_offset_calibrator:main',
        ],
    },
)
