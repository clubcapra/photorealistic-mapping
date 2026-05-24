from pathlib import Path

from setuptools import find_packages, setup

package_name = 'rove_tuning_orchestrator'


def _data_files(subdir, glob='*'):
    base = Path(subdir)
    if not base.exists():
        return []
    return [(
        f'share/{package_name}/{base}',
        [str(p) for p in base.glob(glob) if p.is_file()],
    )]


data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
]
data_files += _data_files('config/search_spaces', '*.yaml')


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'optuna>=3.4', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Iliana',
    maintainer_email='iliana.dc@windo.cleaning',
    description='Two-phase sim+real RTAB-Map tuner with file-based distributed Optuna.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Top-level orchestrator.
            'tune = rove_tuning_orchestrator.orchestrator:main',
            # Workers — pull trials from study, evaluate, push results.
            'worker = rove_tuning_orchestrator.worker:main',
            # Promote phase-1 (sim) top-K into phase-2 (real) study.
            'promote = rove_tuning_orchestrator.promote:main',
            # Launch optuna-dashboard against this project's studies.
            'dashboard = rove_tuning_orchestrator.dashboard:main',
            # Standalone: compare two rtabmap.db files to a reference.
            'compare_to_reference = rove_tuning_orchestrator.reference_compare:main',
        ],
    },
)
