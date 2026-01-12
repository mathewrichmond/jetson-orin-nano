# Standard library
from glob import glob
import os

# Third-party
from setuptools import setup

package_name = "nvblox_processor"

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
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Mathew Richmond",
    maintainer_email="mathewrichmond@gmail.com",
    description="ROS 2 node for nvblox 3D mapping and TSDF processing",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nvblox_processor_node = nvblox_processor.nvblox_processor_node:main",
        ],
    },
)
