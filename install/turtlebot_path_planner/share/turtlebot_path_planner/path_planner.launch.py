from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'trajectory',
            default_value='circle',
            description='Desired trajectory (circle, square, infinity, star)'
        ),
        Node(
            package='turtlebot_path_planner',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'trajectory': LaunchConfiguration('trajectory')
            }]
        ),
    ])
