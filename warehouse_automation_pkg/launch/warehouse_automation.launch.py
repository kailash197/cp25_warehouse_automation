from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare the launch argument with default value
    env_arg = DeclareLaunchArgument(
        'env',
        default_value='sim',
        description='Environment configuration: "sim" for simulation or "real" for real hardware'
    )
    print(LaunchConfiguration('env'))
    
    return LaunchDescription([
        env_arg,

        Node(
            package='warehouse_automation_pkg',
            executable='glocalization_as_node',
            output='screen',
            name='glocalization_as_node',
            parameters=[{'env': LaunchConfiguration('env')}]
        ),

        Node(
            package='warehouse_automation_pkg',
            executable='client_glocalization_as_node',
            output='screen',
            name='client_glocalization_as_node',
            parameters=[{'env': LaunchConfiguration('env')}]
        )
    ])
