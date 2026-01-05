from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.substitutions import FindPackageShare


def _render_nav2_params(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower()
    map_yaml = LaunchConfiguration('map').perform(context)

    namespace_key = f'/{namespace}' if namespace else ''

    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'config',
        'nav2_params.yaml.in',
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / 'nav2_params.yaml'

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NAMESPACE__', namespace_key)
    content = content.replace('__USE_SIM_TIME__', use_sim_time)
    content = content.replace('__MAP_YAML__', map_yaml)
    output_file.write_text(content, encoding='utf-8')

    return []


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')

    nav2_params = PathJoinSubstitution([
        TextSubstitution(text='/tmp/mm_bringup/'),
        prefix,
        TextSubstitution(text='nav2_params.yaml'),
    ])

    nav2_bringup_share = FindPackageShare('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_share,
            '/launch/bringup_launch.py',
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true',
            'slam': slam,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        OpaqueFunction(function=_render_nav2_params),
        nav2_launch,
    ])
