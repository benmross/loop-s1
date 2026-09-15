from glob import glob

from setuptools import find_packages, setup

package_name = 'loop_s1_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Ross',
    maintainer_email='ben.m.ross08@gmail.com',
    description='Autonomous waypoint navigation for the UMD Loop S1 challenge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = loop_s1_nav.navigator:main',
            'waypoint_manager = loop_s1_nav.waypoint_manager:main',
        ],
    },
)
