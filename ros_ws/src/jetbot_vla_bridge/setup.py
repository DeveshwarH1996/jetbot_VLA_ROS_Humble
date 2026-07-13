from setuptools import setup

package_name = 'jetbot_vla_bridge'

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
    maintainer='Deveshwar',
    maintainer_email='deveshwarh@gmail.com',
    description='VLA server client bridge for JetBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_client_bridge = jetbot_vla_bridge.vla_client_bridge:main',
        ],
    },
)
