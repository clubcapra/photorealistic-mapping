from setuptools import setup

package_name = 'lio_sam_wrapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='IMU unit normalization node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_wrapper = lio_sam_wrapper.imu_wrapper:main',
        ],
    },
)
