"""Follow an AprilTag or a person, on top of a running robot (or sim).

robot.launch.py includes this with follow:=tag or follow:=person. On its own,
next to a robot.launch.py that already relays the camera (foxglove:=true, the default):
  ros2 launch th_autonomy follow.launch.py target:=person
Without the Foxglove stack, start the camera relay here too:
  ros2 launch th_autonomy follow.launch.py target:=tag camera_url:=http://localhost:8080/stream
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_follower(context):
    def arg(name):
        return context.launch_configurations[name]
    target = arg('target')
    config = arg('config') or FindPackageShare('th_autonomy').perform(context) + \
        f'/config/{target}_follower.yaml'
    actions = []
    if arg('camera_url'):
        actions.append(Node(
            package='th_foxglove', executable='mjpeg_camera', name='mjpeg_camera',
            parameters=[{'url': arg('camera_url'), 'rotation': int(arg('camera_rotation'))}],
            output='screen'))
    actions.append(Node(
        package='th_autonomy', executable=f'{target}_follower', name=f'{target}_follower',
        parameters=[config], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target', default_value='tag', choices=['tag', 'person'], description='What to follow'),
        DeclareLaunchArgument(
            'config', default_value='',
            description="Parameters; '' = config/<target>_follower.yaml"),
        DeclareLaunchArgument(
            'camera_url', default_value='',
            description="Start an MJPEG relay for this URL; '' = use an existing /camera/image/compressed"),
        DeclareLaunchArgument(
            'camera_rotation', default_value='180',
            description='Degrees clockwise the relay turns frames (walker camera is mounted upside '
                        'down). The relay rotates for every subscriber, so the follower configs keep '
                        'camera.rotation 0'),
        OpaqueFunction(function=launch_follower),
    ])
