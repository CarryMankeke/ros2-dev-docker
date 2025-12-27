from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_prefix = LaunchConfiguration('arm_prefix')
    arm_scale = LaunchConfiguration('arm_scale')
    controllers_file = LaunchConfiguration('controllers_file')
    planning_config = LaunchConfiguration('planning_config')
    planning_scene_config = LaunchConfiguration('planning_scene_config')
    kinematics_config = LaunchConfiguration('kinematics_config')
    servo_config = LaunchConfiguration('servo_config')
    srdf_file = LaunchConfiguration('srdf_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    mm_bringup_share = FindPackageShare('mm_bringup')
    mm_arm_share = FindPackageShare('mm_arm_description')

    arm_xacro = PathJoinSubstitution([mm_arm_share, 'urdf', 'mm_arm.urdf.xacro'])
    srdf_path = PathJoinSubstitution([mm_bringup_share, 'config', srdf_file])
    default_rviz = PathJoinSubstitution([mm_bringup_share, 'rviz', 'mm_display.rviz'])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            arm_xacro,
            ' prefix:=', arm_prefix,
            ' scale:=', arm_scale,
            ' controllers_file:=', PathJoinSubstitution([mm_bringup_share, 'config', 'arm_controllers.yaml']),
        ]),
        value_type=str,
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        remappings=[('joint_states', '/mm_arm/joint_states')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            {
                'robot_description_semantic': ParameterValue(
                    Command(['cat', srdf_path]),
                    value_type=str,
                )
            },
            PathJoinSubstitution([mm_bringup_share, 'config', controllers_file]),
            PathJoinSubstitution([mm_bringup_share, 'config', planning_config]),
            PathJoinSubstitution([mm_bringup_share, 'config', planning_scene_config]),
            PathJoinSubstitution([mm_bringup_share, 'config', kinematics_config]),
        ],
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        output='screen',
        remappings=[('joint_states', '/mm_arm/joint_states')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            PathJoinSubstitution([mm_bringup_share, 'config', servo_config]),
            PathJoinSubstitution([mm_bringup_share, 'config', controllers_file]),
            PathJoinSubstitution([mm_bringup_share, 'config', kinematics_config]),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mm_arm',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('arm_prefix', default_value='mm_arm_'),
        DeclareLaunchArgument('arm_scale', default_value='1.0'),
        DeclareLaunchArgument('controllers_file', default_value='moveit_controllers.yaml'),
        DeclareLaunchArgument('planning_config', default_value='moveit_planning.yaml'),
        DeclareLaunchArgument('planning_scene_config', default_value='moveit_planning_scene.yaml'),
        DeclareLaunchArgument('kinematics_config', default_value='moveit_kinematics.yaml'),
        DeclareLaunchArgument('servo_config', default_value='moveit_servo.yaml'),
        DeclareLaunchArgument('srdf_file', default_value='mm_arm.srdf'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        robot_state_publisher,
        move_group_node,
        servo_node,
        rviz,
    ])
