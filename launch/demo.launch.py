# ROS2 launch file for bringing up the CHAMP robot with teleoperation and state publishing

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    champ_config_launch = os.path.join(
        get_package_share_directory('champ_config'),
        'launch',
        'bringup.launch.py'
    )
    champ_description_urdf = os.path.join(
        get_package_share_directory('champ_description'),
        'urdf',
        'champ_arm.urdf'
    )
    champ_teleop_launch = os.path.join(
        get_package_share_directory('champ_teleop'),
        'launch',
        'teleop.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(champ_config_launch),
            launch_arguments={
                'description_file': champ_description_urdf,
                'has_imu': 'false',
                'rviz': 'true'
            }.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arm_state_publisher',
            output='screen',
            parameters=[{
                'use_tf_static': True,
                'publish_frequency': 200
            }],
            remappings=[
                ('joint_states', 'arm/joint_states')
            ]
        ),
        Node(
            package='chicken_head',
            executable='chicken_head_node',
            name='chicken_head',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(champ_teleop_launch),
            launch_arguments={
                'joy': 'true'
            }.items()
        )
    ])

