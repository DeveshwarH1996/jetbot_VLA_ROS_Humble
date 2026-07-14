import os
from glob import glob
from setuptools import setup

package_name = 'jetbot_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deveshwar',
    maintainer_email='deveshwarh@gmail.com',
    description='Base ROS2 nodes for Waveshare JetBot AI Kit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = jetbot_base.motor_driver:main',
            'joy_controller = jetbot_base.joy_controller:main',
        ],
    },
)
