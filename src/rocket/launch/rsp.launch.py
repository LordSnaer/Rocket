import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('articubot_one'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Joint State Publisher GUI
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2 node with config
    rviz_config_file = os.path.join(
        get_package_share_directory('articubot_one'), 'config', 'rviz_config.rviz'
    )
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RPLidar A2M12 front launch
    rplidar_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m12_launch_front.py')
        )
    )

    # RPLidar A2M12 rear launch
    rplidar_rear_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m12_launch_rear.py')
        )
    )

    # BNO055 IMU node
    bno055_params = os.path.join(
        get_package_share_directory('bno055'), 'config', 'bno055_params_i2c.yaml'
    )
    node_bno055 = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_params]
    )

    laser_filter_front = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_front',
        output='screen',
        parameters=[{
                'filter1': {
                    'name': 'range_filter',
                    'type': 'laser_filters/LaserScanRangeFilter',
                    'params': {
                        'use_message_range_limits': False,
                        'lower_threshold': 0.2,
                        'upper_threshold': 5.0,
                        'lower_replacement_value': float('nan'),
                        'upper_replacement_value': float('nan')
                    }
                },
                'filter2': {
                    'name': 'angle_block_1',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.119467787,  # -6.845 degrees
                        'upper_angle': 0.119467787    # +6.845 degrees
                    }
                },
                'filter3': {
                    'name': 'angle_block_2',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 0.328907,  # +18.845 degrees
                        'upper_angle': 0.503963   # +28.915 degrees
                    }
                },
                'filter4': {
                    'name': 'angle_block_3',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 2.142915,  # +122.8 degrees
                        'upper_angle': 2.465452   # +141.2 degrees
                    }
                },
                'filter5': {
                    'name': 'angle_block_4',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.503963,  # -18.845 degrees
                        'upper_angle': -0.328907   # -28.915 degrees
                    }
                },
                'filter6': {
                    'name': 'angle_block_5',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -2.465452,  # -141.2 degrees
                        'upper_angle': -2.142915   # -122.8 degrees
                    }
                }
            }],
        remappings=[
            ('scan', '/scan_front_raw'),
            ('scan_filtered', '/scan_front')
        ]
    )

    laser_filter_rear = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_rear',
        output='screen',
        parameters=[{
                'filter1': {
                    'name': 'range_filter',
                    'type': 'laser_filters/LaserScanRangeFilter',
                    'params': {
                        'use_message_range_limits': False,
                        'lower_threshold': 0.2,
                        'upper_threshold': 5.0,
                        'lower_replacement_value': float('nan'),
                        'upper_replacement_value': float('nan')
                    }
                },
                'filter2': {
                    'name': 'angle_block_1',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.119467787,  # -6.845 degrees
                        'upper_angle': 0.119467787    # +6.845 degrees
                    }
                },
                'filter3': {
                    'name': 'angle_block_2',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 0.328907,  # +18.845 degrees
                        'upper_angle': 0.503963   # +28.915 degrees
                    }
                },
                'filter4': {
                    'name': 'angle_block_3',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 2.142915,  # +122.8 degrees
                        'upper_angle': 2.465452   # +141.2 degrees
                    }
                },
                'filter5': {
                    'name': 'angle_block_4',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.503963,  # -18.845 degrees
                        'upper_angle': -0.328907   # -28.915 degrees
                    }
                },
                'filter6': {
                    'name': 'angle_block_5',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -2.465452,  # -141.2 degrees
                        'upper_angle': -2.142915   # -122.8 degrees
                    }
                }
            }],
        remappings=[
            ('scan', '/scan_rear_raw'),
            ('scan_filtered', '/scan_rear')
        ]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2,
        rplidar_front_launch,
        rplidar_rear_launch,
        node_bno055,
        laser_filter_front,
        laser_filter_rear,
    ])

