from glob import glob

from setuptools import setup

package_name = 'drone_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='StevenFAU',
    maintainer_email='230911905+StevenFAU@users.noreply.github.com',
    description='Waypoint navigation and ArUco marker approach for an ArduPilot quadcopter.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hello_flight = drone_mission.hello_flight:main',
            'waypoint_navigator = drone_mission.waypoint_navigator:main',
        ],
    },
)
