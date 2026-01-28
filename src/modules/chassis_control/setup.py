# Standard library
from glob import glob
import os

# Third-party
from setuptools import setup

package_name = "chassis_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mathew Richmond",
    maintainer_email="mathewrichmond@gmail.com",
    description="Chassis control module for dual-compute robotics platform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_controller_node = chassis_control.chassis_controller_node:main",
            "imu_processor_node = chassis_control.imu_processor_node:main",
            "calibration_manager_node = chassis_control.calibration_manager_node:main",
        ],
    },
)
