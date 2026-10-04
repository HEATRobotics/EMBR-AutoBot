from setuptools import find_packages, setup

package_name = 'embr_teleoperation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maison Gulyas',
    maintainer_email='maison.personal03@gmail.com',
    description="Combined Teleoperation of capstan and drivetrain. The combined movement controls of EMBR.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "tele = embr_teleoperation.node_teleoperation:main",
        ],
    },
)
