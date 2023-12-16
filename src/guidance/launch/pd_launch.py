from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dongk_controller",
            executable="pd_controller",
            name="custom_pd_control",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_param": 10.0}
            ]
        )
    ])