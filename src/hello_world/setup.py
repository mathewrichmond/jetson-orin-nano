from setuptools import setup
import os
from glob import glob

package_name = 'hello_world'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mathew Richmond',
    maintainer_email='mathewrichmond@gmail.com',
    description='Hello World example for Isaac robot system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world_node = hello_world.hello_world_node:main',
        ],
    },
)
