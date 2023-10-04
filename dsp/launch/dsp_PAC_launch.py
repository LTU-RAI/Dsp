from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter
import os


def generate_launch_description():
    name_space = os.getlogin()
    return LaunchDescription([
        Node(
            package='dsp',
            namespace=name_space,
            executable='dsp',
            name=name_space + '_dsp',
            parameters=[
                Parameter("base_link_frame_id", name_space + "/body"),
                Parameter("frame_id", "map")
            ]
        )
    ])
