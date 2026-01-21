from setuptools import find_packages, setup

package_name = "sensor_sync"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Isaac Robot",
    maintainer_email="isaac@example.com",
    description="Sensor synchronization and Kalman filtering - publishes synchronized sensor data as single source of truth",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_sync_node = sensor_sync.sensor_sync_node:main",
            "hardware_sync_generator_node = sensor_sync.hardware_sync_generator_node:main",
        ],
    },
)
