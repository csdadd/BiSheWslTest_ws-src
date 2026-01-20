from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_model_viewer',
            executable='robot_model_viewer',
            name='robot_model_viewer',
            output='screen',
            emulate_tty=True
        )
    ])