from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'medbot_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Netsanet',
    maintainer_email='netsanet@ethio-medbot.com',
    description='Mission control for Medical Delivery Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_manager = medbot_mission.delivery_manager:main',
            'waypoint_publisher = medbot_mission.waypoint_publisher:main',
            'status_monitor = medbot_mission.status_monitor:main',
            'emergency_stop = medbot_mission.emergency_stop:main',
            'obstacle_detector = medbot_mission.obstacle_detector:main',
        ],
    },
)
