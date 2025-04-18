from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rangenet_ros2',
            executable='rangenet_node',
            name='rangenet_node',
            parameters=[{
                'model': 'src/rangenet_ros2/models/darknet53/',
                'input_topic': '/velodyne_points',
                'output_topic': '/semantic_points'
            }]
        )
    ])
