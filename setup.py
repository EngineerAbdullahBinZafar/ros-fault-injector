from setuptools import setup
import os
from glob import glob

package_name = 'ros_fault_injector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EngineerAbdullahBinZafar',
    maintainer_email='abz.king.1.9.2003@gmail.com',
    description='Chaos Engineering for ROS2: A tool to inject faults, delays, and noise into robotic systems.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'injector_node = ros_fault_injector.injector_node:main',
        ],
    },
)
