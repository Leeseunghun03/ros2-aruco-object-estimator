from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_estimator',
            executable='object_estimator',
            name='object_estimator_node',
            output='screen'
        ),
    ])