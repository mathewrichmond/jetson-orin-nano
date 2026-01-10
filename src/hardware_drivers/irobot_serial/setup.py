from setuptools import setup
import os
from glob import glob

package_name = 'irobot_serial'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Mathew Richmond',
    maintainer_email='mathewrichmond@gmail.com',
    description='ROS 2 node for iRobot Create/Roomba serial communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'irobot_serial_node = irobot_serial.irobot_serial_node:main',
        ],
    },
)
