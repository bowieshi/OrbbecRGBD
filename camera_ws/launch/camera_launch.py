from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            # namespace='camera1',
            executable='talker',
            # name='sim'
        ),
        Node(
            package='camera',
            # namespace='turtlesim2',
            executable='listener',
            # name='sim'
        )
    ])
