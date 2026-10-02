from pathlib import Path

from setuptools import find_packages, setup

package_name = 'embr_capstan'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[
            ('share/' + package_name + '/' + str(directory),
             [str(file) for file in directory.glob('*.md')])
            for directory in sorted(Path('docs').rglob('*'))
            if directory.is_dir() and any(directory.glob('*.md'))
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maison Gulyas',
    maintainer_email='maison.personal03@gmail.com',
    description='EMBR arm actuation through the Eagle Power Motor, Planetary Gearbox, and Capstan.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "cp_helper = embr_capstan.node_capstan_helper:main",
        ],
    },
)
