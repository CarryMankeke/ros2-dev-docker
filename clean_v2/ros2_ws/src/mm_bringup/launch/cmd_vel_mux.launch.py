# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mux = LaunchConfiguration('use_mux')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')

    mux_config = PathJoinSubstitution([
        FindPackageShare('mm_bringup'),
        'config',
        'cmd_vel_mux.yaml.in',
    ])

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=namespace,
        output='screen',
        parameters=[mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', cmd_vel_out)],
        condition=IfCondition(use_mux),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_mux', default_value='true'),
        DeclareLaunchArgument('cmd_vel_out', default_value='omni_wheel_controller/cmd_vel'),
        twist_mux,
    ])
