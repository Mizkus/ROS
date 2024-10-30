import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    delay = LaunchConfiguration('delay', default='2.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='2.0',
            description='Time delay in seconds for the second turtle to follow the first turtle'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        ExecuteProcess(
            cmd=['ros2','service','call','/spawn','turtlesim/srv/Spawn', '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
            output='screen'
        ),
        Node(
            package='ex03',
            executable='turtle1_broadcaster',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}]
        ),
        Node(
            package='ex03',
            executable='turtle2_broadcaster',
            name='broadcaster2',
            parameters=[{'turtlename': 'turtle2'}]
        ),
        Node(
            package='ex03',
            executable='turtle2_listener',
            name='listener',
            parameters=[{'delay': delay}]
        ),
    ])
