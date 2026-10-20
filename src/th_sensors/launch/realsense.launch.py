"""Launch the vendor RealSense driver with Texas Humanoids defaults."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    sensors_share = Path(get_package_share_directory('th_sensors'))
    driver_share = Path(get_package_share_directory('realsense2_camera'))
    with (sensors_share / 'config' / 'realsense.yaml').open() as config:
        defaults = yaml.safe_load(config)

    # Declare defaults here so user-supplied arguments retain precedence. Leave
    # the vendor's other arguments (serial_no, usb_port_id, etc.) available too.
    arguments = [
        DeclareLaunchArgument(
            name,
            default_value=str(value).lower() if isinstance(value, bool) else str(value),
            description=f'RealSense {name} setting',
        )
        for name, value in defaults.items()
    ]
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(driver_share / 'launch' / 'rs_launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name) for name in defaults
        }.items(),
    )
    return LaunchDescription([*arguments, driver])
