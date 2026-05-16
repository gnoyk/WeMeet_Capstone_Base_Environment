from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    dynamic_tracker = Node(
        package='my_valet_parking',
        executable='dynamic_tracker_node',
        name='dynamic_tracker_node',
        output='screen',
        parameters=[{
            'scan_topic':           '/limo1/scan',
            'odom_topic':           '/limo1/odom',
            'target_topic':         '/limo1/target_slot',
            'clear_count_fraction': 0.20,
            'clear_max_points':     30,
            'clear_hold_s':         2.0,
        }],
    )

    limo_prepare = Node(
        package='parking_pkg',
        executable='limo_prepare_node',
        name='limo_prepare_node',
        namespace='/limo1',
        output='screen',
        remappings=[
            ('odom',          '/limo1/odom'),
            ('scan',          '/limo1/scan'),
            ('target_slot',   '/limo1/target_slot'),
            ('cmd_vel',       '/limo1/cmd_vel'),
            ('parking_ready', '/limo1/parking_ready'),
        ],
    )

    limo_parking = Node(
        package='parking_pkg',
        executable='limo_parking_node',
        name='limo_parking_node',
        namespace='/limo1',
        output='screen',
        remappings=[
            ('odom',          '/limo1/odom'),
            ('scan',          '/limo1/scan'),
            ('target_slot',   '/limo1/target_slot'),
            ('cmd_vel',       '/limo1/cmd_vel'),
            ('parking_ready', '/limo1/parking_ready'),
        ],
    )

    # ---- Teleop nodes (commented out by default; run manually or uncomment one) ----
    #
    # limo2_arrow_teleop = Node(
    #     package='my_valet_parking',
    #     executable='limo2_arrow_teleop',
    #     name='limo2_arrow_teleop',
    #     output='screen',
    #     prefix='xterm -e',
    # )

    vehicle_selector = Node(
        package='my_valet_parking',
        executable='vehicle_selector_node',
        name='vehicle_selector_node',
        output='screen',
        respawn=False,
    )

    return LaunchDescription([
        dynamic_tracker,
        limo_prepare,
        limo_parking,
        vehicle_selector,
    ])
