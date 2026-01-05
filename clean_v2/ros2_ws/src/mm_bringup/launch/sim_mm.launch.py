from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
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


# Autor: Camilo Soto Villegas | Contacto: camilo.soto.v@usach.cl | Proyecto: clean_v2
def _render_mm_controllers(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    namespace_key = f'/{namespace}' if namespace else ''
    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'config',
        'mm_controllers.yaml.in',
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / 'mm_controllers.yaml'

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NAMESPACE__', namespace_key)
    output_file.write_text(content, encoding='utf-8')

    return []


def _render_rviz_config(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    namespace_key = f'/{namespace}' if namespace else ''
    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'rviz',
        'mm_display.rviz.in',
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / 'mm_display.rviz'

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NS__', namespace_key)
    output_file.write_text(content, encoding='utf-8')

    return []


def _set_lidar_bridge_arg(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    topic = f'/{namespace}/scan' if namespace else '/scan'
    return [
        SetLaunchConfiguration(
            'lidar_bridge_arg',
            f'{topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        )
    ]


def _create_camera_bridge(context):
    sim = LaunchConfiguration('sim').perform(context).lower()
    if sim in ('false', '0'):
        return []

    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    ns_prefix = f'/{namespace}' if namespace else ''
    camera_prefix = f'{ns_prefix}/camera'
    bridge_args = [
        f'{camera_prefix}/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{camera_prefix}/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{camera_prefix}/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{camera_prefix}/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{camera_prefix}/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{camera_prefix}/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{camera_prefix}/rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{camera_prefix}/rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{camera_prefix}/ee/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{camera_prefix}/ee/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]
    remaps = [
        (f'{camera_prefix}/front/camera_info', f'{camera_prefix}/front/camera_info_raw'),
        (f'{camera_prefix}/left/camera_info', f'{camera_prefix}/left/camera_info_raw'),
        (f'{camera_prefix}/right/camera_info', f'{camera_prefix}/right/camera_info_raw'),
        (f'{camera_prefix}/rear/camera_info', f'{camera_prefix}/rear/camera_info_raw'),
        (f'{camera_prefix}/ee/camera_info', f'{camera_prefix}/ee/camera_info_raw'),
    ]

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=bridge_args,
            remappings=remaps,
        )
    ]


def _create_lidar_bridge(context):
    sim = LaunchConfiguration('sim').perform(context).lower()
    enable = LaunchConfiguration('enable_lidar_bridge').perform(context).lower()
    if sim in ('false', '0') or enable in ('false', '0'):
        return []

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            output='screen',
            arguments=[LaunchConfiguration('lidar_bridge_arg')],
        )
    ]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_lidar_bridge = LaunchConfiguration('enable_lidar_bridge')
    headless = LaunchConfiguration('headless')

    arm_x = LaunchConfiguration('arm_x')
    arm_y = LaunchConfiguration('arm_y')
    arm_z = LaunchConfiguration('arm_z')
    arm_roll = LaunchConfiguration('arm_roll')
    arm_pitch = LaunchConfiguration('arm_pitch')
    arm_yaw = LaunchConfiguration('arm_yaw')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')

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

    robot_xacro = PathJoinSubstitution([
        FindPackageShare('mm_robot_description'),
        'urdf',
        'mm_robot.urdf.xacro',
    ])
    controllers_file = PathJoinSubstitution([
        TextSubstitution(text='/tmp/mm_bringup/'),
        prefix,
        TextSubstitution(text='mm_controllers.yaml'),
    ])
    rviz_config = PathJoinSubstitution([
        TextSubstitution(text='/tmp/mm_bringup/'),
        prefix,
        TextSubstitution(text='mm_display.rviz'),
    ])

    default_world = PathJoinSubstitution([mm_bringup_share, 'worlds', 'minimal.world.sdf'])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,
            ' prefix:=', prefix,
            ' namespace:=', namespace,
            ' controllers_file:=', controllers_file,
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
        condition=IfCondition(sim),
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
        condition=IfCondition(sim),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        condition=IfCondition(sim),
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

    omni_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'omni_wheel_controller',
            '--controller-manager', controller_manager_ns,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager', controller_manager_ns,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'gripper_trajectory_controller',
            '--controller-manager', controller_manager_ns,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '30',
        ],
    )

    start_jsb = TimerAction(period=3.0, actions=[base_jsb])
    start_omni = TimerAction(period=5.0, actions=[omni_controller])
    start_arm = TimerAction(period=7.0, actions=[arm_controller])
    start_gripper = TimerAction(period=9.0, actions=[gripper_controller])
    camera_frame_republisher = Node(
        package='mm_bringup',
        executable='camera_frame_republisher.py',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'prefix': prefix}],
        condition=IfCondition(sim),
    )
    start_rviz = TimerAction(
        period=11.0,
        actions=[
            GroupAction(
                actions=[
                    SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
                    SetEnvironmentVariable(name='QT_XCB_GL_INTEGRATION', value='none'),
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments=['-d', rviz_config],
                    ),
                ]
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gz_args', default_value='-r -v 4 -s --headless-rendering'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_lidar_bridge', default_value='true'),
        DeclareLaunchArgument('arm_x', default_value='0.0'),
        DeclareLaunchArgument('arm_y', default_value='0.0'),
        DeclareLaunchArgument('arm_z', default_value='0.05'),
        DeclareLaunchArgument('arm_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_x', default_value='0.20'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.15'),
        SetEnvironmentVariable(
            name='GZ_SIM_HEADLESS',
            value='1',
            condition=IfCondition(headless),
        ),
        OpaqueFunction(function=_render_mm_controllers),
        OpaqueFunction(function=_render_rviz_config),
        OpaqueFunction(function=_set_lidar_bridge_arg),
        OpaqueFunction(function=_create_camera_bridge),
        OpaqueFunction(function=_create_lidar_bridge),
        gz_launch,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        start_jsb,
        start_omni,
        start_arm,
        start_gripper,
        camera_frame_republisher,
        start_rviz,
    ])
