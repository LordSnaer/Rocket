"""Navigate in an existing (saved) map.

Same as slam.launch.py, except:
  - slam_toolbox is replaced with localization_launch.py (map_server + AMCL)
  - takes a `map` arg pointing to the saved .yaml map file

Run with the default map path:
    ros2 launch articubot_one nav.launch.py
Or override the map:
    ros2 launch articubot_one nav.launch.py map:=/home/rocket/maps/my_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    map_yaml = LaunchConfiguration('map')

    pkg_path = os.path.join(get_package_share_directory('articubot_one'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rplidar_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m12_launch_front.py')
        )
    )

    rplidar_rear_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m12_launch_rear.py')
        )
    )

    laser_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('dual_laser_merger'), 'demo_laser_merger.launch.py')
        )
    )

    # Localization (map_server + AMCL) instead of slam_toolbox
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
        }.items(),
    )

    # nav2 stack (same as slam.launch.py)
    # Wrapped in a TimerAction so localization_launch's lifecycle_manager has
    # time to fully bring up map_server + amcl before nav2's manager starts.
    # Without this delay, on a Jetson the two lifecycle_managers race at
    # startup and one of them (usually localization) deadlocks.
    #
    # map_subscribe_transient_local: navigation_launch.py defaults this arg
    # to 'false', which overrides our YAML via RewrittenYaml. We need 'true'
    # so the global_costmap's static layer can receive map_server's latched
    # /map message.
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_path, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map_subscribe_transient_local': 'true',
                }.items(),
            )
        ]
    )

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

    nodes_dir = '/home/rocket/rocket_ws/src/rocket/nodes'

    odrive_can_test = ExecuteProcess(
        cmd=['python3', os.path.join(nodes_dir, 'odrive_can_test.py')],
        output='screen'
    )

    odom_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(nodes_dir, 'odom_publisher.py')],
        output='screen'
    )

    goal_restamp = ExecuteProcess(
        cmd=['python3', os.path.join(nodes_dir, 'goal_restamp.py')],
        output='screen'
    )

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
                        'lower_angle': -0.119467787,
                        'upper_angle': 0.119467787
                    }
                },
                'filter3': {
                    'name': 'angle_block_2',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 0.285274,
                        'upper_angle': 0.547596
                    }
                },
                'filter4': {
                    'name': 'angle_block_3',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 2.099282,
                        'upper_angle': 2.509085
                    }
                },
                'filter5': {
                    'name': 'angle_block_4',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.547596,
                        'upper_angle': -0.285274
                    }
                },
                'filter6': {
                    'name': 'angle_block_5',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -2.509085,
                        'upper_angle': -2.099282
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
                        'lower_angle': -0.119467787,
                        'upper_angle': 0.119467787
                    }
                },
                'filter3': {
                    'name': 'angle_block_2',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 0.285274,
                        'upper_angle': 0.547596
                    }
                },
                'filter4': {
                    'name': 'angle_block_3',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': 2.099282,
                        'upper_angle': 2.509085
                    }
                },
                'filter5': {
                    'name': 'angle_block_4',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -0.547596,
                        'upper_angle': -0.285274
                    }
                },
                'filter6': {
                    'name': 'angle_block_5',
                    'type': 'laser_filters/LaserScanAngularBoundsFilterInPlace',
                    'params': {
                        'lower_angle': -2.509085,
                        'upper_angle': -2.099282
                    }
                }
            }],
        remappings=[
            ('scan', '/scan_rear_raw'),
            ('scan_filtered', '/scan_rear')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'map',
            default_value='/home/rocket/maps/my_map.yaml',
            description='Full path to the saved map .yaml to load'),

        node_robot_state_publisher,
        rplidar_front_launch,
        rplidar_rear_launch,
        laser_filter_front,
        laser_filter_rear,
        laser_merger_launch,
        localization_launch,
        nav2_launch,
        goal_restamp,
        can_setup,
        start_can_nodes,
    ])
