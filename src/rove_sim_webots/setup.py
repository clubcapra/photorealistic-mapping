from pathlib import Path

from setuptools import find_packages, setup

package_name = 'rove_sim_webots'


def _data_files(subdir, glob='*'):
    base = Path(subdir)
    if not base.exists():
        return []
    return [
        (f'share/{package_name}/{base}', [str(p) for p in base.glob(glob) if p.is_file()]),
    ]


data_files = [
    ('share/ament_index/resource_index/packages',
     [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
]
data_files += _data_files('launch', '*.py')
data_files += _data_files('worlds', '*.wbt')
data_files += _data_files('worlds', '*.wbproj')
data_files += _data_files('protos', '*.proto')
data_files += _data_files('urdf', '*.urdf')
data_files += _data_files('urdf', '*.xacro')
data_files += _data_files('config/trajectories', '*.yaml')


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Iliana',
    maintainer_email='iliana.dc@windo.cleaning',
    description='Webots simulation for the Rove rover, producing RTAB-Map-ready data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Scripted run: drive a trajectory in sim, record a bag, optionally build a rtabmap db.
            'scripted_run = rove_sim_webots.scripted_runner:main',
            # Webots ROS 2 driver plugin entry (loaded via URDF <plugin> tag).
            # The actual class is referenced from the URDF, not invoked here.
        ],
    },
)
