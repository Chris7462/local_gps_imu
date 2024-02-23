from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    gps_shift_node = Node(
        package='gps_imu_node',
        executable='gps_shift_node',
        name='gps_shift_node'
    )

    return LaunchDescription([
        gps_shift_node
    ])
