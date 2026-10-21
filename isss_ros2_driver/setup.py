from glob import glob

from setuptools import find_packages, setup

PACKAGE = 'isss_ros2_driver'

setup(
    name=PACKAGE,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PACKAGE}']),
        (f'share/{PACKAGE}', ['package.xml']),
        (f'share/{PACKAGE}/launch', glob('launch/*.launch.py')),
        (f'share/{PACKAGE}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROS 2 driver for the DeepVision ISSS (thin wrapper around isss_core)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isss_ros2_driver_node = isss_ros2_driver.node:main',
        ],
    },
)
