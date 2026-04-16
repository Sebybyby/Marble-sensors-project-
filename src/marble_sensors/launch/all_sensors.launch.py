"""
Launch all MARBLE sensor nodes.

Override serial ports and network addresses with ROS 2 launch arguments:

  ros2 launch marble_sensors all_sensors.launch.py \
      sbe37_port:=/dev/ttyUSB0 \
      aquadopp_port:=/dev/ttyUSB1 \
      weather_port:=/dev/ttyUSB2 \
      motus_port:=/dev/ttyUSB3 \
      rbr_port:=/dev/ttyUSB4 \
      s2cr_host:=192.168.0.100 \
      oculus_host:=192.168.2.4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------------------------------------------
    # Launch arguments (allows overriding from the command line)
    # ----------------------------------------------------------------
    args = [
        DeclareLaunchArgument('sbe37_port',    default_value='/dev/ttyUSB0',   description='SBE37 serial port'),
        DeclareLaunchArgument('aquadopp_port', default_value='/dev/ttyUSB1',   description='Aquadopp serial port'),
        DeclareLaunchArgument('weather_port',  default_value='/dev/ttyUSB2',   description='Weather station serial port'),
        DeclareLaunchArgument('motus_port',    default_value='/dev/ttyUSB3',   description='MOTUS serial port'),
        DeclareLaunchArgument('rbr_port',      default_value='/dev/ttyUSB4',   description='RBRcoda3 serial port'),
        DeclareLaunchArgument('s2cr_host',     default_value='192.168.0.100',  description='S2CR modem IP address'),
        DeclareLaunchArgument('oculus_host',   default_value='192.168.2.4',    description='Oculus sonar IP address'),
    ]

    # ----------------------------------------------------------------
    # Nodes
    # ----------------------------------------------------------------
    nodes = [
        Node(
            package='marble_sensors',
            executable='sbe37_sip',
            name='sbe37_sip',
            parameters=[{'port': LaunchConfiguration('sbe37_port')}],
            output='screen',
        ),
        Node(
            package='marble_sensors',
            executable='aquadopp_profiler',
            name='aquadopp_profiler',
            parameters=[{'port': LaunchConfiguration('aquadopp_port')}],
            output='screen',
        ),
        Node(
            package='marble_sensors',
            executable='s2cr_modem',
            name='s2cr_modem',
            parameters=[{'host': LaunchConfiguration('s2cr_host')}],
            output='screen',
        ),
        Node(
            package='marble_sensors',
            executable='weather_station',
            name='weather_station',
            parameters=[{'port': LaunchConfiguration('weather_port')}],
            output='screen',
        ),
        Node(
            package='marble_sensors',
            executable='motus_wave',
            name='motus_wave',
            parameters=[{'port': LaunchConfiguration('motus_port')}],
            output='screen',
        ),
        Node(
            package='marble_sensors',
            executable='oculus_sonar',
            name='oculus_sonar',
            parameters=[{'host': LaunchConfiguration('oculus_host')}],
            output='screen',
        ),
        Node(
            package='marble_sensors',
            executable='rbrcoda3',
            name='rbrcoda3',
            parameters=[{'port': LaunchConfiguration('rbr_port')}],
            output='screen',
        ),
    ]

    return LaunchDescription(args + nodes)
