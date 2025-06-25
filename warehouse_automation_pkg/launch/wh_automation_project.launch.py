from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='true = simulation, false = real robot'
    )

    # Get launch arg
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch file paths
    localization_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('localization_server'),
            'launch',
            'localization.launch.py'
        ])
    )

    pathplanner_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('path_planner_server'),
            'launch',
            'pathplanner.launch.py'
        ])
    )

    glocalization_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('warehouse_automation_pkg'),
            'launch',
            'glocalization_action_server.launch.py'
        ])
    )

    # Localization launch (map switches based on use_sim_time)
    localization_sim = IncludeLaunchDescription(
        localization_launch,
        launch_arguments={'map_file': 'warehouse_map_keepout_sim_project.yaml'}.items(),
        condition=IfCondition(use_sim_time)
    )

    localization_real = IncludeLaunchDescription(
        localization_launch,
        launch_arguments={'map_file': 'warehouse_map_keepout_real_project.yaml'}.items(),
        condition=UnlessCondition(use_sim_time)
    )

    # Path planner (10s delay after localization)
    delayed_pathplanner = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                pathplanner_launch,
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # Glocalization (15s delay after path planner = 25s total)
    delayed_glocalization_sim = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                glocalization_launch,
                launch_arguments={'env': 'sim'}.items(),
                condition=IfCondition(use_sim_time)
            )
        ]
    )

    delayed_glocalization_real = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                glocalization_launch,
                launch_arguments={'env': 'real'}.items(),
                condition=UnlessCondition(use_sim_time)
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,

        # Start localization immediately
        localization_sim,
        localization_real,

        # Delay: path planner
        delayed_pathplanner,

        # Delay: glocalization (sim or real)
        delayed_glocalization_sim,
        delayed_glocalization_real
    ])
