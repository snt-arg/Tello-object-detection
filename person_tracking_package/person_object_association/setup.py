from setuptools import find_packages, setup

package_name = 'person_object_association'

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
    description='Maps each person to objects near the person',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'associator_node = person_object_association.person_object_association:main',
        ],
    },
)
