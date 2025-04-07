from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_tracking_for_llm'

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
    description='This package is a tello drone person tracker that must be coupled with a LLM drone command sending package to run',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector_llm_node = person_tracking_for_llm.detect_all:main',
            'pilot_person_selector_llm_node = person_tracking_for_llm.tracking_trigger:main',
            'person_tracker_llm_node = person_tracking_for_llm.track_person:main',
            'pilot_person_drawer_llm_node = person_tracking_for_llm.drawing_target_person:main',
            'video_interface_node = person_tracking_for_llm.video_interface:main',
            'landmarks_filter_node = person_tracking_for_llm.landmarks_filter:main',
            'test_llm = person_tracking_for_llm.publish_test:main'
        ],
    },
)
