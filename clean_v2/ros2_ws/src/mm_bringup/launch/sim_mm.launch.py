# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2
from pathlib import Path
import shlex

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
    rviz_mode = LaunchConfiguration('rviz_mode').perform(context).strip().lower()
    if rviz_mode == 'verify':
        template_name = 'mm_verify.rviz.in'
    elif rviz_mode == 'display':
        template_name = 'mm_display.rviz.in'
    else:
        template_name = 'mm_verify.rviz.in'
    template_path = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'rviz',
        template_name,
    ]).perform(context)

    output_dir = Path('/tmp/mm_bringup') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / template_name.replace('.in', '')

    content = Path(template_path).read_text(encoding='utf-8')
    content = content.replace('__PREFIX__', prefix)
    content = content.replace('__NS__', namespace_key)
    output_file.write_text(content, encoding='utf-8')

    return [SetLaunchConfiguration('rviz_config', str(output_file))]


def _set_gz_args(context):
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    base_args = LaunchConfiguration('gz_args').perform(context).strip()
    tokens = shlex.split(base_args) if base_args else ['-r', '-v', '4']
    if headless:
        if '-s' not in tokens:
            tokens.append('-s')
        if '--headless-rendering' not in tokens:
            tokens.append('--headless-rendering')
    else:
        tokens = [token for token in tokens if token not in ('-s', '--headless-rendering')]
    return [SetLaunchConfiguration('gz_args', ' '.join(tokens))]


def _set_lidar_bridge_arg(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    topic = f'/{namespace}/scan_raw' if namespace else '/scan_raw'
    return [
        SetLaunchConfiguration(
            'lidar_bridge_arg',
            f'{topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        )
    ]


def _create_imu_bridge(context):
    sim = LaunchConfiguration('sim').perform(context).lower()
    enable = LaunchConfiguration('enable_imu_bridge').perform(context).lower()
    if sim in ('false', '0') or enable in ('false', '0'):
        return []

    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    topic = f'/{namespace}/imu_raw' if namespace else '/imu_raw'
    node_name = f'{namespace}_imu_bridge' if namespace else 'imu_bridge'
    
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=node_name,
            namespace=namespace,
            output='screen',
            arguments=[f'{topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        )
    ]


def _create_ee_imu_bridge(context):
    sim = LaunchConfiguration('sim').perform(context).lower()
    enable = LaunchConfiguration('enable_ee_imu_bridge').perform(context).lower()
    if sim in ('false', '0') or enable in ('false', '0'):
        return []

    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    topic = f'/{namespace}/imu/ee_raw' if namespace else '/imu/ee_raw'
    node_name = f'{namespace}_ee_imu_bridge' if namespace else 'ee_imu_bridge'

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=node_name,
            namespace=namespace,
            output='screen',
            arguments=[f'{topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
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
    rviz_config = LaunchConfiguration('rviz_config')

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
            ' enable_imu:=', LaunchConfiguration('enable_imu'),
            ' enable_ee_imu:=', LaunchConfiguration('enable_ee_imu'),
            ' ee_imu_x:=', LaunchConfiguration('ee_imu_x'),
            ' ee_imu_y:=', LaunchConfiguration('ee_imu_y'),
            ' ee_imu_z:=', LaunchConfiguration('ee_imu_z'),
            ' ee_imu_roll:=', LaunchConfiguration('ee_imu_roll'),
            ' ee_imu_pitch:=', LaunchConfiguration('ee_imu_pitch'),
            ' ee_imu_yaw:=', LaunchConfiguration('ee_imu_yaw'),
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

    rviz_visual_descriptions = Node(
        package='mm_bringup',
        executable='rviz_visual_descriptions.py',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'prefix': prefix},
            {'namespace': namespace},
            {'arm_x': arm_x},
            {'arm_y': arm_y},
            {'arm_z': arm_z},
            {'arm_roll': arm_roll},
            {'arm_pitch': arm_pitch},
            {'arm_yaw': arm_yaw},
            {'enable_lidar': enable_lidar},
            {'lidar_x': lidar_x},
            {'lidar_y': lidar_y},
            {'lidar_z': lidar_z},
            {'enable_ee_imu': LaunchConfiguration('enable_ee_imu')},
            {'ee_imu_x': LaunchConfiguration('ee_imu_x')},
            {'ee_imu_y': LaunchConfiguration('ee_imu_y')},
            {'ee_imu_z': LaunchConfiguration('ee_imu_z')},
            {'ee_imu_roll': LaunchConfiguration('ee_imu_roll')},
            {'ee_imu_pitch': LaunchConfiguration('ee_imu_pitch')},
            {'ee_imu_yaw': LaunchConfiguration('ee_imu_yaw')},
        ],
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
    imu_frame_republisher = Node(
        package='mm_bringup',
        executable='imu_frame_republisher.py',
        name='imu_frame_republisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'prefix': prefix},
            {'frame_id': 'imu_link'},
            {'src_topic': 'imu_raw'},
            {'dst_topic': 'imu'},
        ],
        condition=IfCondition(sim),
    )
    ee_imu_frame_republisher = Node(
        package='mm_bringup',
        executable='imu_frame_republisher.py',
        name='ee_imu_frame_republisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'prefix': prefix},
            {'frame_id': 'ee_imu_link'},
            {'src_topic': 'imu/ee_raw'},
            {'dst_topic': 'imu/ee'},
        ],
        condition=IfCondition(LaunchConfiguration('enable_ee_imu_bridge')),
    )
    lidar_frame_republisher = Node(
        package='mm_bringup',
        executable='lidar_frame_republisher.py',
        name='lidar_frame_republisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'prefix': prefix},
            {'frame_id': 'lidar_link'},
            {'src_topic': 'scan_raw'},
            {'dst_topic': 'scan'},
        ],
        condition=IfCondition(sim),
    )
    odom_relay = Node(
        package='mm_bringup',
        executable='odom_relay.py',
        namespace=namespace,
        output='screen',
        arguments=[
            '--input-topic', 'omni_wheel_controller/odom',
            '--output-topic', 'odom',
        ],
        condition=IfCondition(sim),
    )
    start_rviz = TimerAction(
        period=11.0,
        actions=[
            GroupAction(
                actions=[
                    SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
                    SetEnvironmentVariable(name='QT_XCB_GL_INTEGRATION', value='none'),
                    SetEnvironmentVariable(name='GALLIUM_DRIVER', value='llvmpipe'),
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments=['-d', rviz_config],
                        additional_env={
                            'LIBGL_ALWAYS_SOFTWARE': '1',
                            'QT_XCB_GL_INTEGRATION': 'none',
                            'GALLIUM_DRIVER': 'llvmpipe',
                        },
                    ),
                ]
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_mode', default_value='verify'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gz_args', default_value='-r -v 4'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_lidar_bridge', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_imu_bridge', default_value='true'),
        DeclareLaunchArgument('enable_ee_imu', default_value='true'),
        DeclareLaunchArgument('enable_ee_imu_bridge', default_value='true'),
        DeclareLaunchArgument('arm_x', default_value='0.0'),
        DeclareLaunchArgument('arm_y', default_value='0.0'),
        DeclareLaunchArgument('arm_z', default_value='0.02'),
        DeclareLaunchArgument('arm_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_x', default_value='0.20'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.125'),
        DeclareLaunchArgument('ee_imu_x', default_value='0.03'),
        DeclareLaunchArgument('ee_imu_y', default_value='0.0'),
        DeclareLaunchArgument('ee_imu_z', default_value='0.02'),
        DeclareLaunchArgument('ee_imu_roll', default_value='0.0'),
        DeclareLaunchArgument('ee_imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('ee_imu_yaw', default_value='0.0'),
        SetEnvironmentVariable(
            name='GZ_SIM_HEADLESS',
            value='1',
            condition=IfCondition(headless),
        ),
        OpaqueFunction(function=_render_mm_controllers),
        OpaqueFunction(function=_render_rviz_config),
        OpaqueFunction(function=_set_gz_args),
        OpaqueFunction(function=_set_lidar_bridge_arg),
        OpaqueFunction(function=_create_camera_bridge),
        OpaqueFunction(function=_create_lidar_bridge),
        OpaqueFunction(function=_create_imu_bridge),
        OpaqueFunction(function=_create_ee_imu_bridge),
        gz_launch,
        clock_bridge,
        robot_state_publisher,
        rviz_visual_descriptions,
        spawn_robot,
        start_jsb,
        start_omni,
        start_arm,
        start_gripper,
        camera_frame_republisher,
        imu_frame_republisher,
        ee_imu_frame_republisher,
        lidar_frame_republisher,
        odom_relay,
        start_rviz,
    ])
