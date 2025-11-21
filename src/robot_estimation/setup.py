from setuptools import setup
import os
from glob import glob

package_name = 'robot_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot estimation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = robot_estimation.motor_controller_node:main',
            'imu_filter_node = robot_estimation.imu_filter_node:main',
            'complementary_filter_node = robot_estimation.complementary_filter_node:main',
            'odometry_node = robot_estimation.odometry_node:main',
            'estimator_test_node = robot_estimation.estimator_test_node:main',
        ],
    },
)