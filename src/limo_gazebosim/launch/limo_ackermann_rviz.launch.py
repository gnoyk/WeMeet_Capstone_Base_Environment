from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

import os


def generate_launch_description():
    # [REQ-5] 헤드리스 모드: GAZEBO_HEADLESS=1 환경변수 또는 headless:=true 인수로 GUI 없이 실행
    _env_headless = os.environ.get('GAZEBO_HEADLESS', '0')
    _headless_default = 'true' if _env_headless not in ('', '0', 'false', 'False') else 'false'

    limo_car_launch = os.path.join(
        get_package_share_directory('limo_car'),
        'launch',
        'ackermann.launch.py',
    )
    parking_world = PathJoinSubstitution([
        get_package_share_directory('limo_gazebosim'),
        'worlds',
        'parking_lot_scaled_vehicles.world',
    ])
    package_models = os.path.join(
        get_package_share_directory('limo_gazebosim'),
        'models',
    )
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py',
    )
    default_rviz_config = os.path.join(
        get_package_share_directory('limo_car'),
        'rviz',
        'gazebo.rviz',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value=_headless_default,
        description='헤드리스 모드 (GUI 없음). GAZEBO_HEADLESS=1 환경변수로도 설정 가능.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config,
        description='Absolute path to the RViz config file.',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=parking_world,
        description='Full path to the Gazebo world file.',
    )
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            os.pathsep,
            os.path.expanduser('~/.gazebo/models'),
            os.pathsep,
            package_models,
        ],
    )
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(limo_car_launch),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    # [REQ-5] headless=true면 gui:=false로 Gazebo 실행 (gzserver만)
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': 'true',
            'extra_gazebo_args': '--physics-threads 4',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': 'false',
            'extra_gazebo_args': '--physics-threads 4',
        }.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '-10.0',
            '-Y', '0.0',
        ],
        output='screen',
    )
    # [REQ-5] headless 모드일 때 RViz 실행 안 함
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    return LaunchDescription([
        headless_arg,
        rviz_arg,
        world_arg,
        gazebo_model_path,
        robot_description,
        gazebo_gui,
        gazebo_headless,
        spawn_entity,
        rviz_node,
    ])
