from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    clock_mode = LaunchConfiguration('clock_mode')
    launch_sim = LaunchConfiguration('launch_sim')
    launch_display = LaunchConfiguration('launch_display')
    input_mode = LaunchConfiguration('input')
    teleop_mode = LaunchConfiguration('teleop_mode')

    mm_bringup_share = FindPackageShare('mm_bringup')
    sim_launch = PathJoinSubstitution([mm_bringup_share, 'launch', 'sim.launch.py'])
    display_launch = PathJoinSubstitution([mm_bringup_share, 'launch', 'display.launch.py'])
    joy_params = PathJoinSubstitution([mm_bringup_share, 'config', 'joy_teleop.yaml'])

    use_sim_time = PythonExpression(["'", clock_mode, "' == 'sim'"])

    use_fake_joint_states = PythonExpression(
        ["'true' if ('", launch_sim, "' == 'false') else 'false'"]
    )
    use_joint_state_gui = PythonExpression(
        [
            "'true' if (('",
            input_mode,
            "' == 'gui') and ('",
            launch_sim,
            "' == 'false')) else 'false'",
        ]
    )
    publish_base_to_arm_tf = PythonExpression(
        ["'true' if ('", launch_sim, "' == 'false') else 'false'"]
    )

    is_sim = PythonExpression(["'", launch_sim, "' == 'true'"])
    is_display = PythonExpression(["'", launch_display, "' == 'true'"])
    is_input_esp32 = PythonExpression(["'", input_mode, "' == 'esp32'"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(is_sim),
    )

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch),
        launch_arguments={
            'use_fake_joint_states': use_fake_joint_states,
            'use_joint_state_gui': use_joint_state_gui,
            'publish_base_to_arm_tf': publish_base_to_arm_tf,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(is_display),
    )

    teleop = Node(
        package='mm_bringup',
        executable='joy_teleop.py',
        output='screen',
        parameters=[joy_params, {'mode': teleop_mode}],
        condition=IfCondition(is_input_esp32),
    )

    return LaunchDescription([
        DeclareLaunchArgument('clock_mode', default_value='sim'),
        DeclareLaunchArgument('launch_sim', default_value='true'),
        DeclareLaunchArgument('launch_display', default_value='true'),
        DeclareLaunchArgument('input', default_value='none'),
        DeclareLaunchArgument('teleop_mode', default_value='hybrid'),
        DeclareLaunchArgument('use_fake_joint_states', default_value='false'),
        DeclareLaunchArgument('use_joint_state_gui', default_value='false'),
        DeclareLaunchArgument('publish_base_to_arm_tf', default_value='false'),
        sim,
        display,
        teleop,
    ])
