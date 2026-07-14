import os
from glob import glob
from setuptools import setup

package_name = 'jetbot_nav'

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
    description='Traditional Nav2-based navigation and mode arbitration for JetBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_plane_projector = jetbot_nav.ground_plane_projector:main',
            'mode_arbiter = jetbot_nav.mode_arbiter:main',
        ],
    },
)
