from setuptools import find_packages, setup

package_name = 'person_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maeri',
    maintainer_email='maevachekma@gmail.com',
    description='Autonomous person tracker for Tello Drone',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'all_detected_node = person_tracking.detect_all:main',
            'trigger_node = person_tracking.tracking_trigger:main',
            'tracker_node = person_tracking.track_person:main',
            'test = person_tracking.publish_test:main', 
        ],
    },
)
