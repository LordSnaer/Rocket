import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    # Dual laser merger
    laser_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dual_laser_merger'), 'demo_laser_merger.launch.py')
        )
    )

    # slam_toolbox (online async) — uses /merged from the laser merger
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # Bring up can0 at 250 kbps. Non-blocking: if can0 is already UP we skip
    # sudo entirely; otherwise use `sudo -n` so a missing NOPASSWD rule fails
    # fast instead of hanging the launch on an invisible password prompt.
    can_setup = ExecuteProcess(
        cmd=['bash', '-c',
             'if ip -br link show can0 2>/dev/null | grep -q "UP"; then '
             '  echo "[can_setup] can0 already up — skipping"; exit 0; '
             'fi; '
             'sudo -n ip link set can0 down 2>/dev/null; '
             'sudo -n ip link set can0 type can bitrate 250000 && '
             'sudo -n ip link set can0 up || '
             'echo "[can_setup] FAILED (need NOPASSWD sudo for /sbin/ip, or bring can0 up manually)"'],
        output='screen'
    )

    # nodes/ is not installed by CMakeLists, so reference the source tree directly
    nodes_dir = '/home/rocket/rocket_ws/src/rocket/nodes'

    odrive_can_test = ExecuteProcess(
        cmd=['python3', os.path.join(nodes_dir, 'odrive_can_test.py')],
        output='screen'
    )

    odom_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(nodes_dir, 'odom_publisher.py')],
        output='screen'
    )

    # Only start CAN-using nodes after can0 is up
    start_can_nodes = RegisterEventHandler(
        OnProcessExit(
            target_action=can_setup,
            on_exit=[odrive_can_test, odom_publisher],
        )
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
                        'lower_angle': 0.285274,  # +16.346 degrees
                        'upper_angle': 0.547596   # +31.376 degrees
                    }
                },
                'filter4': {
                    'name': 'angle_block_3',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 2.099282,  # +120.282 degrees
                        'upper_angle': 2.509085   # +143.760 degrees
                    }
                },
                'filter5': {
                    'name': 'angle_block_4',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.547596,  # -31.376 degrees
                        'upper_angle': -0.285274   # -16.346 degrees
                    }
                },
                'filter6': {
                    'name': 'angle_block_5',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -2.509085,  # -143.760 degrees
                        'upper_angle': -2.099282   # -120.282 degrees
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
                        'lower_angle': 0.285274,  # +16.346 degrees
                        'upper_angle': 0.547596   # +31.376 degrees
                    }
                },
                'filter4': {
                    'name': 'angle_block_3',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 2.099282,  # +120.282 degrees
                        'upper_angle': 2.509085   # +143.760 degrees
                    }
                },
                'filter5': {
                    'name': 'angle_block_4',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.547596,  # -31.376 degrees
                        'upper_angle': -0.285274   # -16.346 degrees
                    }
                },
                'filter6': {
                    'name': 'angle_block_5',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -2.509085,  # -143.760 degrees
                        'upper_angle': -2.099282   # -120.282 degrees
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
        rplidar_front_launch,
        rplidar_rear_launch,
        laser_filter_front,
        laser_filter_rear,
        laser_merger_launch,
        slam_launch,
        can_setup,
        start_can_nodes,
    ])
