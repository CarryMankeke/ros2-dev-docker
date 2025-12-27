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
    base_prefix = LaunchConfiguration('base_prefix')
    arm_prefix = LaunchConfiguration('arm_prefix')
    base_scale = LaunchConfiguration('base_scale')
    arm_scale = LaunchConfiguration('arm_scale')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    model_cache_dir = LaunchConfiguration('model_cache_dir')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    nav2_map = LaunchConfiguration('nav2_map')
    nav2_params = LaunchConfiguration('nav2_params')
    nav2_bt = LaunchConfiguration('nav2_bt')
    launch_moveit = LaunchConfiguration('launch_moveit')
    moveit_srdf = LaunchConfiguration('moveit_srdf')
    moveit_controllers = LaunchConfiguration('moveit_controllers')
    moveit_planning = LaunchConfiguration('moveit_planning')
    moveit_kinematics = LaunchConfiguration('moveit_kinematics')
    moveit_servo = LaunchConfiguration('moveit_servo')
    world = LaunchConfiguration('world')
    base_x = LaunchConfiguration('base_x')
    base_y = LaunchConfiguration('base_y')
    base_z = LaunchConfiguration('base_z')
    base_roll = LaunchConfiguration('base_roll')
    base_pitch = LaunchConfiguration('base_pitch')
    base_yaw = LaunchConfiguration('base_yaw')
    arm_x = LaunchConfiguration('arm_x')
    arm_y = LaunchConfiguration('arm_y')
    arm_z = LaunchConfiguration('arm_z')
    arm_roll = LaunchConfiguration('arm_roll')
    arm_pitch = LaunchConfiguration('arm_pitch')
    arm_yaw = LaunchConfiguration('arm_yaw')

    mm_bringup_share = FindPackageShare('mm_bringup')
    sim_launch = PathJoinSubstitution([mm_bringup_share, 'launch', 'sim.launch.py'])
    display_launch = PathJoinSubstitution([mm_bringup_share, 'launch', 'display.launch.py'])
    nav2_launch = PathJoinSubstitution([mm_bringup_share, 'launch', 'nav2.launch.py'])
    moveit_launch = PathJoinSubstitution([mm_bringup_share, 'launch', 'moveit.launch.py'])
    joy_params = PathJoinSubstitution([mm_bringup_share, 'config', 'joy_teleop.yaml'])
    world_file = PathJoinSubstitution([mm_bringup_share, 'worlds', 'warehouse.sdf'])
    nav2_map_default = PathJoinSubstitution([mm_bringup_share, 'maps', 'blank.yaml'])
    nav2_params_default = PathJoinSubstitution([mm_bringup_share, 'config', 'nav2_params.yaml'])
    moveit_srdf_default = PathJoinSubstitution([mm_bringup_share, 'config', 'mm_arm.srdf'])
    moveit_controllers_default = PathJoinSubstitution([mm_bringup_share, 'config', 'moveit_controllers.yaml'])
    moveit_planning_default = PathJoinSubstitution([mm_bringup_share, 'config', 'moveit_planning.yaml'])
    moveit_kinematics_default = PathJoinSubstitution([mm_bringup_share, 'config', 'moveit_kinematics.yaml'])
    moveit_servo_default = PathJoinSubstitution([mm_bringup_share, 'config', 'moveit_servo.yaml'])

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
    publish_base_to_arm_tf_sim = LaunchConfiguration('publish_base_to_arm_tf')

    is_sim = PythonExpression(["'", launch_sim, "' == 'true'"])
    is_display = PythonExpression(["'", launch_display, "' == 'true'"])
    is_input_esp32 = PythonExpression(["'", input_mode, "' == 'esp32'"])
    is_nav2 = PythonExpression(["'", launch_nav2, "' == 'true'"])
    is_moveit = PythonExpression(["'", launch_moveit, "' == 'true'"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gz_verbosity': gz_verbosity,
            'base_prefix': base_prefix,
            'arm_prefix': arm_prefix,
            'base_scale': base_scale,
            'arm_scale': arm_scale,
            'model_cache_dir': model_cache_dir,
            'world': world,
            'base_x': base_x,
            'base_y': base_y,
            'base_z': base_z,
            'base_roll': base_roll,
            'base_pitch': base_pitch,
            'base_yaw': base_yaw,
            'arm_x': arm_x,
            'arm_y': arm_y,
            'arm_z': arm_z,
            'arm_roll': arm_roll,
            'arm_pitch': arm_pitch,
            'arm_yaw': arm_yaw,
            'publish_base_to_arm_tf': publish_base_to_arm_tf_sim,
        }.items(),
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'namespace': 'mm_base',
            'use_sim_time': use_sim_time,
            'map': nav2_map,
            'params_file': nav2_params,
            'autostart': 'true',
            'bt_xml': nav2_bt,
        }.items(),
        condition=IfCondition(is_nav2),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'arm_prefix': arm_prefix,
            'arm_scale': arm_scale,
            'srdf_file': moveit_srdf,
            'controllers_file': moveit_controllers,
            'planning_config': moveit_planning,
            'kinematics_config': moveit_kinematics,
            'servo_config': moveit_servo,
        }.items(),
        condition=IfCondition(is_moveit),
    )

    teleop = Node(
        package='mm_bringup',
        executable='joy_teleop.py',
        output='screen',
        parameters=[joy_params, {'mode': teleop_mode, 'use_sim_time': use_sim_time}],
        condition=IfCondition(is_input_esp32),
    )

    return LaunchDescription([
        DeclareLaunchArgument('clock_mode', default_value='sim'),
        DeclareLaunchArgument('launch_sim', default_value='true'),
        DeclareLaunchArgument('launch_display', default_value='true'),
        DeclareLaunchArgument('launch_nav2', default_value='false'),
        DeclareLaunchArgument('nav2_map', default_value=nav2_map_default),
        DeclareLaunchArgument('nav2_params', default_value=nav2_params_default),
        DeclareLaunchArgument('nav2_bt', default_value=''),
        DeclareLaunchArgument('launch_moveit', default_value='false'),
        DeclareLaunchArgument('moveit_srdf', default_value=moveit_srdf_default),
        DeclareLaunchArgument('moveit_controllers', default_value=moveit_controllers_default),
        DeclareLaunchArgument('moveit_planning', default_value=moveit_planning_default),
        DeclareLaunchArgument('moveit_kinematics', default_value=moveit_kinematics_default),
        DeclareLaunchArgument('moveit_servo', default_value=moveit_servo_default),
        DeclareLaunchArgument('input', default_value='none'),
        DeclareLaunchArgument('teleop_mode', default_value='hybrid'),
        DeclareLaunchArgument('use_fake_joint_states', default_value='false'),
        DeclareLaunchArgument('use_joint_state_gui', default_value='false'),
        DeclareLaunchArgument('publish_base_to_arm_tf', default_value='true'),
        DeclareLaunchArgument('gz_verbosity', default_value='4'),
        DeclareLaunchArgument('model_cache_dir', default_value='/tmp/mm_bringup'),
        DeclareLaunchArgument('base_prefix', default_value='mm_base_'),
        DeclareLaunchArgument('arm_prefix', default_value='mm_arm_'),
        DeclareLaunchArgument('base_scale', default_value='1.0'),
        DeclareLaunchArgument('arm_scale', default_value='1.0'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('base_x', default_value='0.0'),
        DeclareLaunchArgument('base_y', default_value='0.0'),
        DeclareLaunchArgument('base_z', default_value='0.0'),
        DeclareLaunchArgument('base_roll', default_value='0.0'),
        DeclareLaunchArgument('base_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
        DeclareLaunchArgument('arm_x', default_value='0.0'),
        DeclareLaunchArgument('arm_y', default_value='0.0'),
        DeclareLaunchArgument('arm_z', default_value='0.06'),
        DeclareLaunchArgument('arm_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),
        sim,
        display,
        nav2,
        moveit,
        teleop,
    ])
