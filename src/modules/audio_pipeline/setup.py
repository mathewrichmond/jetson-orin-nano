from setuptools import setup

package_name = 'audio_pipeline'

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
    description='Audio pipeline module for feature extraction and speech processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_feature_extractor_node = audio_pipeline.audio_feature_extractor_node:main',
            'speech_recognition_node = audio_pipeline.speech_recognition_node:main',
            'audio_pipeline_node = audio_pipeline.audio_pipeline_node:main',
        ],
    },
)
