from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    robot_description_topic = PathJoinSubstitution([
        TextSubstitution(text='/'),
        namespace,
        TextSubstitution(text='robot_description'),
    ])
    controller_manager_ns = PathJoinSubstitution([
        TextSubstitution(text='/'),
        namespace,
        TextSubstitution(text='controller_manager'),
    ])

    mm_bringup_share = FindPackageShare('mm_bringup')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    base_xacro = PathJoinSubstitution([
        FindPackageShare('mm_base_description'),
        'urdf',
        'mm_base.urdf.xacro',
    ])
    base_controllers = PathJoinSubstitution([
        mm_bringup_share,
        'config',
        'base_controllers.yaml',
    ])

    default_world = PathJoinSubstitution([mm_bringup_share, 'worlds', 'minimal.world.sdf'])

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

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ros_gz_sim_share,
            '/launch/gz_sim.launch.py',
        ]),
        launch_arguments={
            'gz_args': [gz_args, ' ', world],
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', namespace,
            '-topic', robot_description_topic,
            '-z', '0.15',
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    base_jsb = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager_ns,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )
    start_base_jsb = TimerAction(period=3.0, actions=[base_jsb])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gz_args', default_value='-r -v 4 -s --headless-rendering'),
        SetEnvironmentVariable(name='GZ_SIM_HEADLESS', value='1'),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        gz_launch,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        start_base_jsb,
    ])
