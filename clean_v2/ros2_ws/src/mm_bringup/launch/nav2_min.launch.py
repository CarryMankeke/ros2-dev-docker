# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
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


def _render_nav2_rviz(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    namespace_key = f'/{namespace}' if namespace else ''

    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'rviz',
        'mm_nav2.rviz.in',
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / 'mm_nav2.rviz'

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NS__', namespace_key)
    output_file.write_text(content, encoding='utf-8')

    return [SetLaunchConfiguration('rviz_config', str(output_file))]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')

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
            'use_composition': use_composition,
        }.items(),
    )

    rviz_node = GroupAction([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'none'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ], condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=_render_nav2_params),
        OpaqueFunction(function=_render_nav2_rviz),
        nav2_launch,
        rviz_node,
    ])
