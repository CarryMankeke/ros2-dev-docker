from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _render_dual_controllers(context):
    mm_bringup_share = FindPackageShare('mm_bringup').perform(context)
    template_path = Path(mm_bringup_share) / 'config' / 'mm_controllers.yaml.in'

    actions = []
    for key in ('mm1', 'mm2'):
        prefix = LaunchConfiguration(f'{key}_prefix').perform(context)
        namespace = LaunchConfiguration(f'{key}_namespace').perform(context).strip('/')
        namespace_key = f'/{namespace}' if namespace else ''

        output_dir = Path('/tmp/mm_bringup') / prefix
        output_dir.mkdir(parents=True, exist_ok=True)
        output_file = output_dir / 'mm_controllers.yaml'

        content = template_path.read_text(encoding='utf-8')
        content = content.replace('__PREFIX__', prefix)
        content = content.replace('__NAMESPACE__', namespace_key)
        output_file.write_text(content, encoding='utf-8')

    return actions


def _set_lidar_bridge_args(context):
    actions = []
    for key in ('mm1', 'mm2'):
        namespace = LaunchConfiguration(f'{key}_namespace').perform(context).strip('/')
        topic = f'/{namespace}/scan' if namespace else '/scan'
        actions.append(
            SetLaunchConfiguration(
                f'{key}_lidar_bridge_arg',
                f'{topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            )
        )
    return actions


def generate_launch_description():
    mm1_namespace = LaunchConfiguration('mm1_namespace')
    mm2_namespace = LaunchConfiguration('mm2_namespace')
    mm1_prefix = LaunchConfiguration('mm1_prefix')
    mm2_prefix = LaunchConfiguration('mm2_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_lidar_bridge = LaunchConfiguration('enable_lidar_bridge')

    arm_x = LaunchConfiguration('arm_x')
    arm_y = LaunchConfiguration('arm_y')
    arm_z = LaunchConfiguration('arm_z')
    arm_roll = LaunchConfiguration('arm_roll')
    arm_pitch = LaunchConfiguration('arm_pitch')
    arm_yaw = LaunchConfiguration('arm_yaw')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')

    mm1_spawn_x = LaunchConfiguration('mm1_spawn_x')
    mm1_spawn_y = LaunchConfiguration('mm1_spawn_y')
    mm2_spawn_x = LaunchConfiguration('mm2_spawn_x')
    mm2_spawn_y = LaunchConfiguration('mm2_spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    mm_bringup_share = FindPackageShare('mm_bringup')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    robot_xacro = PathJoinSubstitution([
        FindPackageShare('mm_robot_description'),
        'urdf',
        'mm_robot.urdf.xacro',
    ])

    mm1_controllers = PathJoinSubstitution([
        TextSubstitution(text='/tmp/mm_bringup/'),
        mm1_prefix,
        TextSubstitution(text='mm_controllers.yaml'),
    ])
    mm2_controllers = PathJoinSubstitution([
        TextSubstitution(text='/tmp/mm_bringup/'),
        mm2_prefix,
        TextSubstitution(text='mm_controllers.yaml'),
    ])

    default_world = PathJoinSubstitution([mm_bringup_share, 'worlds', 'minimal.world.sdf'])

    mm1_robot_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,
            ' prefix:=', mm1_prefix,
            ' namespace:=', mm1_namespace,
            ' controllers_file:=', mm1_controllers,
            ' arm_x:=', arm_x,
            ' arm_y:=', arm_y,
            ' arm_z:=', arm_z,
            ' arm_roll:=', arm_roll,
            ' arm_pitch:=', arm_pitch,
            ' arm_yaw:=', arm_yaw,
            ' enable_lidar:=', enable_lidar,
            ' lidar_x:=', lidar_x,
            ' lidar_y:=', lidar_y,
            ' lidar_z:=', lidar_z,
        ]),
        value_type=str,
    )

    mm2_robot_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,
            ' prefix:=', mm2_prefix,
            ' namespace:=', mm2_namespace,
            ' controllers_file:=', mm2_controllers,
            ' arm_x:=', arm_x,
            ' arm_y:=', arm_y,
            ' arm_z:=', arm_z,
            ' arm_roll:=', arm_roll,
            ' arm_pitch:=', arm_pitch,
            ' arm_yaw:=', arm_yaw,
            ' enable_lidar:=', enable_lidar,
            ' lidar_x:=', lidar_x,
            ' lidar_y:=', lidar_y,
            ' lidar_z:=', lidar_z,
        ]),
        value_type=str,
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

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    mm1_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge_mm1',
        output='screen',
        arguments=[LaunchConfiguration('mm1_lidar_bridge_arg')],
        condition=IfCondition(enable_lidar_bridge),
    )

    mm2_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge_mm2',
        output='screen',
        arguments=[LaunchConfiguration('mm2_lidar_bridge_arg')],
        condition=IfCondition(enable_lidar_bridge),
    )

    mm1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=mm1_namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': mm1_robot_description}],
    )

    mm2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=mm2_namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': mm2_robot_description}],
    )

    mm1_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', mm1_namespace,
            '-topic', PathJoinSubstitution([
                TextSubstitution(text='/'),
                mm1_namespace,
                TextSubstitution(text='robot_description'),
            ]),
            '-x', mm1_spawn_x,
            '-y', mm1_spawn_y,
            '-z', spawn_z,
        ],
    )

    mm2_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', mm2_namespace,
            '-topic', PathJoinSubstitution([
                TextSubstitution(text='/'),
                mm2_namespace,
                TextSubstitution(text='robot_description'),
            ]),
            '-x', mm2_spawn_x,
            '-y', mm2_spawn_y,
            '-z', spawn_z,
        ],
    )

    mm1_controller_manager = PathJoinSubstitution([
        TextSubstitution(text='/'),
        mm1_namespace,
        TextSubstitution(text='controller_manager'),
    ])
    mm2_controller_manager = PathJoinSubstitution([
        TextSubstitution(text='/'),
        mm2_namespace,
        TextSubstitution(text='controller_manager'),
    ])

    mm1_jsb = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', mm1_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )
    mm2_jsb = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', mm2_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    mm1_omni = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'omni_wheel_controller',
            '--controller-manager', mm1_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )
    mm2_omni = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'omni_wheel_controller',
            '--controller-manager', mm2_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    mm1_arm = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager', mm1_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )
    mm2_arm = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager', mm2_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    mm1_gripper = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'gripper_trajectory_controller',
            '--controller-manager', mm1_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )
    mm2_gripper = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'gripper_trajectory_controller',
            '--controller-manager', mm2_controller_manager,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mm1_namespace', default_value='mm1'),
        DeclareLaunchArgument('mm2_namespace', default_value='mm2'),
        DeclareLaunchArgument('mm1_prefix', default_value='mm1_'),
        DeclareLaunchArgument('mm2_prefix', default_value='mm2_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gz_args', default_value='-r -v 4 -s --headless-rendering'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_lidar_bridge', default_value='true'),
        DeclareLaunchArgument('arm_x', default_value='0.0'),
        DeclareLaunchArgument('arm_y', default_value='0.0'),
        DeclareLaunchArgument('arm_z', default_value='0.30'),
        DeclareLaunchArgument('arm_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_x', default_value='0.20'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.15'),
        DeclareLaunchArgument('mm1_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('mm1_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('mm2_spawn_x', default_value='1.0'),
        DeclareLaunchArgument('mm2_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.15'),
        SetEnvironmentVariable(name='GZ_SIM_HEADLESS', value='1'),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        OpaqueFunction(function=_render_dual_controllers),
        OpaqueFunction(function=_set_lidar_bridge_args),
        gz_launch,
        clock_bridge,
        mm1_lidar_bridge,
        mm2_lidar_bridge,
        mm1_state_publisher,
        mm2_state_publisher,
        mm1_spawn,
        mm2_spawn,
        TimerAction(period=3.0, actions=[mm1_jsb, mm2_jsb]),
        TimerAction(period=5.0, actions=[mm1_omni, mm2_omni]),
        TimerAction(period=7.0, actions=[mm1_arm, mm2_arm]),
        TimerAction(period=9.0, actions=[mm1_gripper, mm2_gripper]),
    ])
