from pathlib import Path

# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _render_base_controllers(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    namespace_key = f'/{namespace}' if namespace else ''
    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'config',
        'base_controllers.yaml.in',
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / 'base_controllers.yaml'

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NAMESPACE__', namespace_key)
    output_file.write_text(content, encoding='utf-8')

    return []


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base_xacro = PathJoinSubstitution([
        FindPackageShare('mm_base_description'),
        'urdf',
        'mm_base.urdf.xacro',
    ])
    base_controllers = PathJoinSubstitution([
        TextSubstitution(text='/tmp/mm_bringup/'),
        prefix,
        TextSubstitution(text='base_controllers.yaml'),
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            base_xacro,
            ' prefix:=', prefix,
            ' namespace:=', namespace,
            ' controllers_file:=', base_controllers,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_render_base_controllers),
        robot_state_publisher,
    ])
