from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join("share", package_name, "config"),glob("config/*"),),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ingrid Maeva Chekam',
    maintainer_email='maevachekma@gmail.com',
    description='Autonomous person tracker for Tello Drone',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector_node = person_tracking.detect_all:main',
            'pilot_person_selector_node = person_tracking.tracking_trigger:main',
            'person_tracker_node = person_tracking.track_person:main',
            'pilot_person_drawer_node = person_tracking.drawing_target_person:main',
            'test = person_tracking.publish_test:main', 
        ],
    },
)
