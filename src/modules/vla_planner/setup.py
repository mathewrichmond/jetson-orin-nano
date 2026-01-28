from setuptools import setup

package_name = 'vla_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Robot',
    maintainer_email='isaac@robot.local',
    description='VLA planner module for Vision-Language-Action model integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_controller_node = vla_planner.vla_controller_node:main',
            'action_executor_node = vla_planner.action_executor_node:main',
            'planner_node = vla_planner.planner_node:main',
        ],
    },
)
