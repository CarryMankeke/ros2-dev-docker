# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rqt = LaunchConfiguration('use_rqt')

    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
        condition=IfCondition(use_rqt),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rqt', default_value='false'),
        rqt_graph,
    ])
