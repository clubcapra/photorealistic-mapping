from glob import glob
import os

from setuptools import setup

package_name = 'rove_color_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*urdf.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='IMU unit normalization node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_wrapper = rove_color_mapping.imu_wrapper:main',
            'lidar_wrapper = rove_color_mapping.lidar_wrapper:main' 
        ],
    },
)
