from setuptools import setup

package_name = 'jetbot_governor'

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
    maintainer='Srinivas',
    maintainer_email='srinivas@example.com',
    description='Predictive safety governor for JetBot VLA proposals',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'predictive_governor = jetbot_governor.predictive_governor:main',
            'mock_lidar_publisher = jetbot_governor.mock_lidar_publisher:main',
        ],
    },
)
