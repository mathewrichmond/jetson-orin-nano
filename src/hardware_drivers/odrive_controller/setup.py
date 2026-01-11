# Standard library
from glob import glob
import os

# Third-party
from setuptools import setup

package_name = "odrive_controller"

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
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Mathew Richmond",
    maintainer_email="mathewrichmond@gmail.com",
    description="ROS 2 node for ODrive motor controller",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odrive_controller_node = odrive_controller.odrive_controller_node:main",
        ],
    },
)
