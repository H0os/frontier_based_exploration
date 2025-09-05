from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_exploration',
            executable='exploration',
            name='exploration_node',
            output='screen',
            parameters=[
                {'weight_w1': 1.0},
                {'weight_w2': 0.5},
                {'weight_w3': 100.0}
            ]
        )
    ])
