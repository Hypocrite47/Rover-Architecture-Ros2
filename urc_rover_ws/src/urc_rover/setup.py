from setuptools import setup
import os
from glob import glob

package_name = 'urc_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        package_name + '.missions',
        package_name + '.navigation',
        package_name + '.sensors',
        package_name + '.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.edu',
    description='URC Rover System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = urc_rover.mission_manager:main',
            'navigation_controller = urc_rover.navigation.navigation_controller:main',
            'camera_handler = urc_rover.sensors.camera_handler:main',
            'gnss_simulator = urc_rover.sensors.gnss_simulator:main',
            'led_indicator = urc_rover.utils.led_indicator:main',
            'science_mission = urc_rover.missions.science_mission:main',
            'delivery_mission = urc_rover.missions.delivery_mission:main',
            'autonomous_nav = urc_rover.missions.autonomous_nav:main',
	        'servicing_mission = urc_rover.missions.servicing_mission:main',
        ],
    },
)
