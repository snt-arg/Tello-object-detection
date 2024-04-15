from setuptools import find_packages, setup

package_name = 'tello'

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
    maintainer='maeri_n',
    maintainer_email='maevachekma@gmail.com',
    description='Package for Tello object detection project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = tello.publisher_member_function:main',
                'listener = tello.subscriber_member_function:main',
		'camera_pub = tello.camera_publisher:main',
		'camera_sub = tello.camera_subscriber:main',
		'detected_pub = tello.sub1:main',
		
        ],
    },
)
