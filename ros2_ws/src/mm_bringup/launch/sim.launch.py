import subprocess
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _generate_robot_files(
    context,
    base_xacro,
    arm_xacro,
    base_controllers_yaml,
    arm_controllers_yaml,
    base_prefix,
    arm_prefix,
    base_scale,
    arm_scale,
    arm_x,
    arm_y,
    arm_z,
    arm_roll,
    arm_pitch,
    arm_yaw,
):
    base_xacro_path = base_xacro.perform(context)
    arm_xacro_path = arm_xacro.perform(context)
    base_controllers_path = base_controllers_yaml.perform(context)
    arm_controllers_path = arm_controllers_yaml.perform(context)
    base_prefix_val = base_prefix.perform(context)
    arm_prefix_val = arm_prefix.perform(context)
    base_scale_val = base_scale.perform(context)
    arm_scale_val = arm_scale.perform(context)
    arm_x_val = arm_x.perform(context)
    arm_y_val = arm_y.perform(context)
    arm_z_val = arm_z.perform(context)
    arm_roll_val = arm_roll.perform(context)
    arm_pitch_val = arm_pitch.perform(context)
    arm_yaw_val = arm_yaw.perform(context)

    with open('/tmp/mm_base.urdf', 'w', encoding='utf-8') as handle:
        subprocess.run(
            [
                'xacro',
                base_xacro_path,
                f'prefix:={base_prefix_val}',
                f'scale:={base_scale_val}',
                f'controllers_file:={base_controllers_path}',
            ],
            check=True,
            stdout=handle,
        )

    with open('/tmp/mm_arm.urdf', 'w', encoding='utf-8') as handle:
        subprocess.run(
            [
                'xacro',
                arm_xacro_path,
                f'prefix:={arm_prefix_val}',
                f'scale:={arm_scale_val}',
                f'controllers_file:={arm_controllers_path}',
            ],
            check=True,
            stdout=handle,
        )

    assembly_sdf = '\n'.join(
        [
            '<?xml version="1.0"?>',
            '<sdf version="1.7">',
            '  <model name="mm_assembly">',
            '    <pose>0 0 0 0 0 0</pose>',
            '    <include>',
            '      <uri>file:///tmp/mm_base.urdf</uri>',
            '      <name>mm_base</name>',
            '      <pose>0 0 0 0 0 0</pose>',
            '    </include>',
            '    <include>',
            '      <uri>file:///tmp/mm_arm.urdf</uri>',
            '      <name>mm_arm</name>',
            f'      <pose>{arm_x_val} {arm_y_val} {arm_z_val} {arm_roll_val} {arm_pitch_val} {arm_yaw_val}</pose>',
            '    </include>',
            '    <joint name="base_arm_fixed" type="fixed">',
            f'      <parent>mm_base::{base_prefix_val}base_link</parent>',
            f'      <child>mm_arm::{arm_prefix_val}root_link</child>',
            '    </joint>',
            '  </model>',
            '</sdf>',
        ]
    )
    with open('/tmp/mm_assembly.sdf', 'w', encoding='utf-8') as handle:
        handle.write(assembly_sdf + '\n')

    return []


def _wait_for_service_and_start(context, service_name, timeout_sec, action_to_start, label):
    deadline = time.monotonic() + float(timeout_sec)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f'Timeout waiting for {label} controller manager service: {service_name}')

        result = subprocess.run(
            ['ros2', 'service', 'wait', service_name, '--timeout', '2'],
            capture_output=True,
        )
        if result.returncode == 0:
            print(f'[{label}] controller manager is ready: {service_name}')
            return [action_to_start]


def _chain_or_shutdown(next_action, label):
    def _handler(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'[{label}] spawner failed with code {event.returncode}.'),
                EmitEvent(event=Shutdown(reason=f'{label} spawner failed')),
            ]
        return [next_action] if next_action is not None else []

    return _handler


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_prefix = LaunchConfiguration('base_prefix')
    arm_prefix = LaunchConfiguration('arm_prefix')
    base_scale = LaunchConfiguration('base_scale')
    arm_scale = LaunchConfiguration('arm_scale')
    world = LaunchConfiguration('world')

    arm_x = LaunchConfiguration('arm_x')
    arm_y = LaunchConfiguration('arm_y')
    arm_z = LaunchConfiguration('arm_z')
    arm_roll = LaunchConfiguration('arm_roll')
    arm_pitch = LaunchConfiguration('arm_pitch')
    arm_yaw = LaunchConfiguration('arm_yaw')

    base_link_frame = PythonExpression(["'", base_prefix, "base_link'"])
    arm_link_frame = PythonExpression(["'", arm_prefix, "root_link'"])

    mm_bringup_share = FindPackageShare('mm_bringup')
    mm_base_share = FindPackageShare('mm_base_description')
    mm_arm_share = FindPackageShare('mm_arm_description')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    base_xacro = PathJoinSubstitution([mm_base_share, 'urdf', 'mm_base.urdf.xacro'])
    arm_xacro = PathJoinSubstitution([mm_arm_share, 'urdf', 'mm_arm.urdf.xacro'])

    base_controllers_yaml = PathJoinSubstitution(
        [mm_bringup_share, 'config', 'base_controllers.yaml']
    )
    arm_controllers_yaml = PathJoinSubstitution(
        [mm_bringup_share, 'config', 'arm_controllers.yaml']
    )

    world_file = PathJoinSubstitution(
        [mm_bringup_share, 'worlds', 'warehouse.sdf']
    )

    generate_files = OpaqueFunction(
        function=_generate_robot_files,
        args=[
            base_xacro,
            arm_xacro,
            base_controllers_yaml,
            arm_controllers_yaml,
            base_prefix,
            arm_prefix,
            base_scale,
            arm_scale,
            arm_x,
            arm_y,
            arm_z,
            arm_roll,
            arm_pitch,
            arm_yaw,
        ],
    )

    base_robot_description_pub = Node(
        package='mm_bringup',
        executable='robot_description_publisher.py',
        namespace='mm_base',
        output='screen',
        parameters=[{'urdf_file': '/tmp/mm_base.urdf'}],
    )

    arm_robot_description_pub = Node(
        package='mm_bringup',
        executable='robot_description_publisher.py',
        namespace='mm_arm',
        output='screen',
        parameters=[{'urdf_file': '/tmp/mm_arm.urdf'}],
    )


    models_path = PathJoinSubstitution([mm_bringup_share, 'models'])
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path,
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ros_gz_sim_share, '/launch/gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world]}.items()
    )

    gz_launch_late = TimerAction(
        period=5.0,
        actions=[gz_launch],
    )

    base_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mm_base',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro ', base_xacro,
                                                          ' prefix:=', base_prefix,
                                                          ' scale:=', base_scale,
                                                          ' controllers_file:=', base_controllers_yaml]),
                                                  value_type=str)}
        ],
    )

    arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mm_arm',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro ', arm_xacro,
                                                          ' prefix:=', arm_prefix,
                                                          ' scale:=', arm_scale,
                                                          ' controllers_file:=', arm_controllers_yaml]),
                                                  value_type=str)}
        ],
    )

    base_to_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_arm_tf',
        output='screen',
        arguments=[
            '--x', arm_x,
            '--y', arm_y,
            '--z', arm_z,
            '--roll', arm_roll,
            '--pitch', arm_pitch,
            '--yaw', arm_yaw,
            '--frame-id', base_link_frame,
            '--child-frame-id', arm_link_frame,
        ],
    )

    base_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/mm_base/mm_base_controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen',
    )

    base_mecanum = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager', '/mm_base/mm_base_controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen',
    )

    arm_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/mm_arm/mm_arm_controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen',
    )

    arm_traj = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager', '/mm_arm/mm_arm_controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen',
    )

    base_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/mm_base/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/mm_base/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mm_base/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/mm_base/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen',
    )

    arm_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/mm_arm/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/mm_arm/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/mm_arm/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_base_mecanum = RegisterEventHandler(
        OnProcessExit(
            target_action=base_jsb,
            on_exit=_chain_or_shutdown(base_mecanum, 'mm_base joint_state_broadcaster'),
        )
    )

    start_arm_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=base_mecanum,
            on_exit=_chain_or_shutdown(arm_jsb, 'mm_base mecanum_drive_controller'),
        )
    )

    start_arm_traj = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_jsb,
            on_exit=_chain_or_shutdown(arm_traj, 'mm_arm joint_state_broadcaster'),
        )
    )

    stop_on_arm_traj_failure = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_traj,
            on_exit=_chain_or_shutdown(None, 'mm_arm arm_trajectory_controller'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('base_prefix', default_value='mm_base_'),
        DeclareLaunchArgument('arm_prefix', default_value='mm_arm_'),
        DeclareLaunchArgument('base_scale', default_value='1.0'),
        DeclareLaunchArgument('arm_scale', default_value='1.0'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('arm_x', default_value='0.0'),
        DeclareLaunchArgument('arm_y', default_value='0.0'),
        DeclareLaunchArgument('arm_z', default_value='0.06'),
        DeclareLaunchArgument('arm_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),

        generate_files,
        base_robot_description_pub,
        arm_robot_description_pub,
        base_to_arm_tf,
        set_gz_resource_path,
        gz_launch_late,
        base_state_publisher,
        arm_state_publisher,
        base_bridge,
        clock_bridge,
        arm_bridge,
        start_base_mecanum,
        start_arm_jsb,
        start_arm_traj,
        stop_on_arm_traj_failure,
    ])
