from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
        FindPackageShare('mm_bringup'),
        'config',
        'base_controllers.yaml',
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
        robot_state_publisher,
    ])
