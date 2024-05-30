import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get package directories
    fusion_bot_dir = get_package_share_directory('fusion_bot')
    depthimage_to_laserscan_dir = get_package_share_directory('depthimage_to_laserscan')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(fusion_bot_dir, 'worlds', 'obstacles.world'),
        description='Path to the world file'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(fusion_bot_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Path to the SLAM toolbox parameters file'
    )

    declare_map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.getcwd(), 'obmap.yaml'),
        description='Path to the map file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )

    # Set environment variable
    set_env_var = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Navigation launch configuration
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # Remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Parameter substitutions
    param_substitutions = {'use_sim_time': use_sim_time, 'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart}, {'node_names': lifecycle_nodes}]
            ),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
            ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings
            ),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': lifecycle_nodes}]
            ),
        ]
    )

    # Include other launch files
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_bot_dir, 'launch', 'launch_sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    depthimage_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(depthimage_to_laserscan_dir, 'launch', 'depthimage_to_laserscan-launch.py'))
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'params_file': LaunchConfiguration('params_file'), 'use_sim_time': 'true'}.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_bot_dir, 'launch', 'localization_launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'), 'use_sim_time': 'true'}.items()
    )

    # Execute standalone commands
    rviz_command = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', 'src/fusion_bot/config/fusion.rviz', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    laser_scan_merger_command = ExecuteProcess(
        cmd=['ros2', 'run', 'laser_merger', 'laser_scan_merger'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_world_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_map_file_arg)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Set environment variables
    ld.add_action(set_env_var)

    # Add included launch files
    ld.add_action(launch_sim)
    ld.add_action(depthimage_to_laserscan_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(localization_launch)

    # Add node groups and processes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(rviz_command)
    ld.add_action(laser_scan_merger_command)

    return ld
