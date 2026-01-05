from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _launch_move_group(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    moveit_share = Path(FindPackageShare('mm_moveit_config').perform(context))

    output_dir = Path('/tmp/mm_moveit') / prefix
    output_dir.mkdir(parents=True, exist_ok=True)

    for template_name in ('moveit_controllers.yaml.in', 'joint_limits.yaml.in'):
        template_path = moveit_share / 'config' / template_name
        output_path = output_dir / template_name.replace('.in', '')
        content = template_path.read_text(encoding='utf-8')
        content = content.replace('__PREFIX__', prefix)
        output_path.write_text(content, encoding='utf-8')

    kinematics = _load_yaml(moveit_share / 'config' / 'kinematics.yaml')
    ompl_planning = _load_yaml(moveit_share / 'config' / 'ompl_planning.yaml')
    planning_scene = _load_yaml(moveit_share / 'config' / 'planning_scene_monitor_parameters.yaml')
    trajectory_execution = _load_yaml(moveit_share / 'config' / 'trajectory_execution.yaml')

    moveit_controllers = _load_yaml(output_dir / 'moveit_controllers.yaml')
    joint_limits = _load_yaml(output_dir / 'joint_limits.yaml')

    robot_xacro = PathJoinSubstitution([
        FindPackageShare('mm_robot_description'),
        'urdf',
        'mm_robot.urdf.xacro',
    ])
    srdf_xacro = PathJoinSubstitution([
        FindPackageShare('mm_arm_description'),
        'srdf',
        'mm_arm.srdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,
            ' prefix:=', prefix,
            ' namespace:=', namespace,
            ' enable_ros2_control:=false',
            ' enable_lidar:=false',
        ]),
        value_type=str,
    )

    robot_description_semantic = ParameterValue(
        Command([
            'xacro ',
            srdf_xacro,
            ' prefix:=', prefix,
        ]),
        value_type=str,
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            ompl_planning,
            moveit_controllers,
            planning_scene,
            trajectory_execution,
        ],
    )

    return [move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mm1'),
        DeclareLaunchArgument('prefix', default_value='mm1_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_move_group),
    ])
