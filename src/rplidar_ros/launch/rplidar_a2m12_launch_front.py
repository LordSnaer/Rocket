#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = True

    laser_filter_config = os.path.join(
        get_package_share_directory('articubot_one'), 'config', 'laser_filter_front.yaml'
    )

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_front',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_41f17b4a955f9047a1e5868ec48d86fa-if00-port0',
                'serial_baudrate': 256000,
                'frame_id': 'laser_frame_front',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            remappings=[
                ('scan', 'scan_front_raw'),
            ],
            sigterm_timeout='10',
            sigkill_timeout='15',
            output='screen'),

#        Node(
#            package='laser_filters',
#            executable='scan_to_scan_filter_chain',
#            name='laser_filter_front',
#            parameters=[laser_filter_config],
#            remappings=[
#                ('scan', '/scan_front_raw'),
#                ('scan_filtered', '/scan_front')
#            ],
#            output='screen'
#        ),
    ])
