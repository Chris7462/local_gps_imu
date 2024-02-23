from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_rotate_node = Node(
        package='gps_imu_node',
        executable='imu_rotate_node',
        name='imu_rotate_node'
    )

    return LaunchDescription([
        imu_rotate_node
    ])
