from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gps_imu_node = Node(
        package='gps_imu_node',
        executable='gps_imu_node',
        name='gps_imu_node'
    )

    return LaunchDescription([
        gps_imu_node
    ])
