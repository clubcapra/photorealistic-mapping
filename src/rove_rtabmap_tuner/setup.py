from glob import glob
import os

from setuptools import setup

package_name = 'rove_rtabmap_tuner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={
        package_name: ['templates/*.tmpl'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'),
         glob(os.path.join(package_name, 'templates', '*.tmpl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iliana',
    maintainer_email='iliana.dc@windo.cleaning',
    description='Automated RTAB-Map parameter tuning harness.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'render_template = rove_rtabmap_tuner.template_renderer:main',
            'run_trial = rove_rtabmap_tuner.trial_runner:main',
            'score_trial = rove_rtabmap_tuner.scoring:main',
            'rank_trials = rove_rtabmap_tuner.scoring:rank_main',
            'optimize = rove_rtabmap_tuner.optimizer:main',
            'analyze_per_bag = rove_rtabmap_tuner.analyze_per_bag:main',
        ],
    },
)
