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


def _render_full_rviz(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    namespace_key = f'/{namespace}' if namespace else ''

    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'rviz',
        'mm_full.rviz.in',
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / 'mm_full.rviz'

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NS__', namespace_key)
    output_file.write_text(content, encoding='utf-8')

    return [SetLaunchConfiguration('rviz_config', str(output_file))]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    use_moveit = LaunchConfiguration('use_moveit')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')

    bringup_share = FindPackageShare('mm_bringup')
    moveit_share = FindPackageShare('mm_moveit_config')

    sim_mm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bringup_share,
            '/launch/sim_mm.launch.py',
        ]),
        launch_arguments={
            'namespace': namespace,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'headless': headless,
            'use_rviz': 'false',
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            moveit_share,
            '/launch/moveit.launch.py',
        ]),
        condition=IfCondition(use_moveit),
        launch_arguments={
            'namespace': namespace,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bringup_share,
            '/launch/nav2_min.launch.py',
        ]),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'namespace': namespace,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'slam': slam,
            'map': map_yaml,
            'use_composition': use_composition,
            'use_rviz': 'false',
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
    ], condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_moveit', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_composition', default_value='False'),
        OpaqueFunction(function=_render_full_rviz),
        sim_mm,
        moveit,
        nav2,
        rviz_node,
    ])
