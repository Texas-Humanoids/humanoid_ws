from setuptools import setup

package_name = 'th_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Texas Humanoids Controls',
    maintainer_email='noreply@github.com',
    description='MuJoCo stand-in for the robot motors and IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = th_sim.mujoco_sim_node:main',
        ],
    },
)
