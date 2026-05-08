
import os
from glob import glob
from setuptools import setup
 
package_name = 'vectornav_udp_bridge'
 
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='VectorNav VN-300 UDP -> ROS2 bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vectornav_udp_node = vectornav_udp_bridge.vectornav_udp_node:main',
        ],
    },
)
