import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _generate_urdf_files(
    context,
    base_xacro,
    arm_xacro,
    base_controllers_yaml,
    arm_controllers_yaml,
    base_prefix,
    arm_prefix,
    base_scale,
    arm_scale,
):
    base_xacro_path = base_xacro.perform(context)
    arm_xacro_path = arm_xacro.perform(context)
    base_controllers_path = base_controllers_yaml.perform(context)
    arm_controllers_path = arm_controllers_yaml.perform(context)
    base_prefix_val = base_prefix.perform(context)
    arm_prefix_val = arm_prefix.perform(context)
    base_scale_val = base_scale.perform(context)
    arm_scale_val = arm_scale.perform(context)

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

    return []


def generate_launch_description():
    use_fake_joint_states = LaunchConfiguration('use_fake_joint_states')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    base_prefix = LaunchConfiguration('base_prefix')
    arm_prefix = LaunchConfiguration('arm_prefix')
    base_scale = LaunchConfiguration('base_scale')
    arm_scale = LaunchConfiguration('arm_scale')
    arm_x = LaunchConfiguration('arm_x')
    arm_y = LaunchConfiguration('arm_y')
    arm_z = LaunchConfiguration('arm_z')
    arm_roll = LaunchConfiguration('arm_roll')
    arm_pitch = LaunchConfiguration('arm_pitch')
    arm_yaw = LaunchConfiguration('arm_yaw')

    base_link_frame = PythonExpression(["'", base_prefix, "base_link'"])
    arm_link_frame = PythonExpression(["'", arm_prefix, "base_link'"])

    mm_bringup_share = FindPackageShare('mm_bringup')
    mm_base_share = FindPackageShare('mm_base_description')
    mm_arm_share = FindPackageShare('mm_arm_description')

    base_xacro = PathJoinSubstitution([mm_base_share, 'urdf', 'mm_base.urdf.xacro'])
    arm_xacro = PathJoinSubstitution([mm_arm_share, 'urdf', 'mm_arm.urdf.xacro'])

    base_controllers_yaml = PathJoinSubstitution(
        [mm_bringup_share, 'config', 'base_controllers.yaml']
    )
    arm_controllers_yaml = PathJoinSubstitution(
        [mm_bringup_share, 'config', 'arm_controllers.yaml']
    )

    default_rviz = PathJoinSubstitution(
        [mm_bringup_share, 'rviz', 'mm_display.rviz']
    )

    base_description = ParameterValue(
        Command([
            'xacro ', base_xacro,
            ' prefix:=', base_prefix,
            ' scale:=', base_scale,
            ' controllers_file:=', base_controllers_yaml
        ]),
        value_type=str,
    )

    arm_description = ParameterValue(
        Command([
            'xacro ', arm_xacro,
            ' prefix:=', arm_prefix,
            ' scale:=', arm_scale,
            ' controllers_file:=', arm_controllers_yaml
        ]),
        value_type=str,
    )

    generate_files = OpaqueFunction(
        function=_generate_urdf_files,
        args=[
            base_xacro,
            arm_xacro,
            base_controllers_yaml,
            arm_controllers_yaml,
            base_prefix,
            arm_prefix,
            base_scale,
            arm_scale,
        ],
    )

    base_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mm_base',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': base_description},
        ],
    )

    arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mm_arm',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': arm_description},
        ],
    )

    base_to_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_arm_tf_display',
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
        condition=IfCondition(use_fake_joint_states),
    )

    arm_joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='mm_arm',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': arm_description},
        ],
        condition=IfCondition(use_fake_joint_states),
    )

    start_rviz = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_joint_states', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('base_prefix', default_value='mm_base_'),
        DeclareLaunchArgument('arm_prefix', default_value='mm_arm_'),
        DeclareLaunchArgument('base_scale', default_value='1.0'),
        DeclareLaunchArgument('arm_scale', default_value='1.0'),
        DeclareLaunchArgument('arm_x', default_value='0.0'),
        DeclareLaunchArgument('arm_y', default_value='0.0'),
        DeclareLaunchArgument('arm_z', default_value='0.06'),
        DeclareLaunchArgument('arm_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),

        generate_files,
        base_state_publisher,
        arm_state_publisher,
        base_to_arm_tf,
        arm_joint_state_gui,
        start_rviz,
    ])
