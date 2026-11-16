from glob import glob

from setuptools import setup

package_name = 'th_foxglove'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/layouts', glob('layouts/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Texas Humanoids Controls',
    maintainer_email='noreply@github.com',
    description='Foxglove dashboard for the robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_monitor = th_foxglove.robot_monitor:main',
            'mjpeg_camera = th_foxglove.mjpeg_camera:main',
        ],
    },
)
