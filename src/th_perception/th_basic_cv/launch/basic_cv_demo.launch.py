"""Run the basic CV node on the bundled sample images, or on a live camera topic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_samples = LaunchConfiguration('use_samples')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_samples', default_value='true',
            description='Publish the bundled sample images. Set false to use a camera.'),
        DeclareLaunchArgument(
            'image_topic', default_value='image_raw',
            description='Image topic the CV node reads, e.g. /image from cam2image.'),
        Node(
            package='th_basic_cv',
            executable='sample_image_publisher',
            condition=IfCondition(use_samples),
            remappings=[('image_raw', image_topic)],
        ),
        Node(
            package='th_basic_cv',
            executable='basic_cv_node',
            remappings=[('image_raw', image_topic)],
        ),
    ])
