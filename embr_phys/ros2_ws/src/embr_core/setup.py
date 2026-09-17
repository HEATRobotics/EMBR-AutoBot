from setuptools import find_packages, setup


package_name = "embr_core"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "IBUS_SETUP.md"]),
    ],
    install_requires=["setuptools", "canopen>=2.3,<3"],
    zip_safe=True,
    maintainer="Maison Gulyas",
    maintainer_email="maison.personal03@gmail.com",
    description="EMBR core nodes and dependencies",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "CANopen = embr.node_canopen_handler:main",
            "drivetrain = embr.node_maxon_drivetrain:main",
            "showcase = embr.node_maxon_single_showcase:main",
            "teleoperation = embr.node_teleoperation:main",
        ],
    },
)
