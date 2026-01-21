from setuptools import find_packages, setup

package_name = "isaac_utils"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Isaac Robot",
    maintainer_email="nano@isaac.local",
    description="Shared utility library for Isaac robot system",
    license="MIT",
    tests_require=["pytest"],
    entry_points={},
)
