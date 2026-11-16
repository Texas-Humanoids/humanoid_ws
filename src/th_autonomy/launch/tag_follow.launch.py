"""AprilTag following on top of a running robot (or sim).

robot.launch.py includes this with tag_follow:=true. On its own, next to a
robot.launch.py that already relays the camera (foxglove:=true, the default):
  ros2 launch th_autonomy tag_follow.launch.py
Without the Foxglove stack, start the camera relay here too:
  ros2 launch th_autonomy tag_follow.launch.py camera_url:=http://localhost:8080/stream
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    camera_url = LaunchConfiguration('camera_url')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('th_autonomy'), 'config', 'tag_follower.yaml']),
            description='tag_follower parameters'),
        DeclareLaunchArgument(
            'camera_url', default_value='',
            description="Start an MJPEG relay for this URL; '' = use an existing /camera/image/compressed"),
        Node(
            package='th_foxglove', executable='mjpeg_camera', name='mjpeg_camera',
            parameters=[{'url': camera_url}],
            condition=IfCondition(PythonExpression(["'", camera_url, "' != ''"])),
            output='screen'),
        Node(
            package='th_autonomy', executable='tag_follower', name='tag_follower',
            parameters=[config], output='screen'),
    ])
