from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    bag_exec = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-r", "1.0", "/data/Kitti/raw/2011_09_29_drive_0071_sync_bag" , "--clock"]
    )

    gps_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gps_imu_node"), "launch", "gps_imu_launch.py"
            ])
        ])
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", join(get_package_share_directory("gps_imu_node"), "rviz", "gps_imu.rviz")]
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        bag_exec,
        gps_imu_launch,
        rviz_node
    ])
