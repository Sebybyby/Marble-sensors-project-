import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'marble_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2'],
    zip_safe=True,
    maintainer='MARBLE Team',
    maintainer_email='contact@marble.com',
    description='ROS 2 nodes for MARBLE project oceanographic sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sbe37_sip        = marble_sensors.sbe37_sip_node:main',
            'aquadopp_profiler = marble_sensors.aquadopp_profiler_node:main',
            's2cr_modem        = marble_sensors.s2cr_modem_node:main',
            'weather_station   = marble_sensors.weather_station_node:main',
            'motus_wave        = marble_sensors.motus_wave_node:main',
            'oculus_sonar      = marble_sensors.oculus_sonar_node:main',
            'rbrcoda3          = marble_sensors.rbrcoda3_node:main',
        ],
    },
)
