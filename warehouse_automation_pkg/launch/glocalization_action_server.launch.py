from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

def generate_launch_description():
    # Declare the launch argument with default value
    env_arg = DeclareLaunchArgument(
        'env',
        default_value='sim',
        description='Environment configuration: "sim" for simulation or "real" for real hardware'
    )

    # Get the value of 'env' at runtime
    env_config = LaunchConfiguration('env')

    return LaunchDescription([
        env_arg,

        # Log the value of the 'env' parameter
        LogInfo(msg=['[glocalization.launch] Environment: ', env_config]),

        Node(
            package='warehouse_automation_pkg',
            executable='glocalization_as_node',
            output='screen',
            name='glocalization_as_node',
            parameters=[{'env': env_config}]
        ),

        # Node(
        #     package='warehouse_automation_pkg',
        #     executable='client_glocalization_as_node',
        #     output='screen',
        #     name='client_glocalization_as_node',
        #     parameters=[{'env': env_config}]
        # )
    ])
