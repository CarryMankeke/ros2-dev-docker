from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    bt_xml = LaunchConfiguration('bt_xml')

    nav2_bringup_share = FindPackageShare('nav2_bringup')
    nav2_bt_share = FindPackageShare('nav2_bt_navigator')
    bringup_launch = PathJoinSubstitution(
        [nav2_bringup_share, 'launch', 'bringup_launch.py']
    )
    default_bt = PathJoinSubstitution(
        [nav2_bt_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml']
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_yaml,
            'default_bt_xml_filename': bt_xml,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm_base'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('mm_bringup'), 'maps', 'blank.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('mm_bringup'), 'config', 'nav2_params.yaml']
            ),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('bt_xml', default_value=default_bt),
        nav2,
    ])
