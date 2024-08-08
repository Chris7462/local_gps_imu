from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    local_gps_imu_node = Node(
        package='local_gps_imu',
        executable='local_gps_imu_node',
        name='local_gps_imu_node'
    )

    return LaunchDescription([
        local_gps_imu_node
    ])
