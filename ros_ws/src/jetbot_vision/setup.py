from setuptools import setup

package_name = 'jetbot_vision'

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
    description='TensorRT accelerated YOLO vision for JetBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = jetbot_vision.camera_node:main',
            'yolo_detector = jetbot_vision.yolo_detector:main',
            'mock_camera_publisher = jetbot_vision.mock_camera_publisher:main',
        ],
    },
)
