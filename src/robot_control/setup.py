from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # เพิ่มโฟลเดอร์ launch เพื่อให้ ROS2 สามารถติดตั้งได้
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='chayakorn.siri@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_node = robot_control.server_node:main',
            'service_node = robot_control.service_node:main',
        ],
    },
)
