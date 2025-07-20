from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter
import os
import getpass


def generate_launch_description():
    name_space = getpass.getuser()
    return LaunchDescription([
        Node(
            package='dsp',
            namespace=name_space,
            executable='dsp',
            name=name_space + '_dsp',
            parameters=[
                Parameter("base_link_frame_id", "/body"),
                Parameter("frame_id", "map"),
                Parameter("map_topic", "/map"),
                Parameter("use_3d", False)
            ]
        )
    ])
