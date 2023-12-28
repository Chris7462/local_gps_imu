from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    bag_exec = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-r", "1.0", "/data/Kitti/raw/2011_09_29_drive_0071_sync_bag" , "--clock"]
    )

    # The TF and URDF of the vehicle
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kitti_urdf"), "launch", "kitti_urdf_launch.py"
            ])
        ])
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", join(get_package_share_directory("ekf_localizer"), "rviz", "ekf_localizer.rviz")]
    )

    gps_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gps_imu_node"), "launch", "gps_imu_launch.py"
            ])
        ])
    )

    ekf_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ekf_localizer"), "launch", "ekf_localizer_launch.py"
            ])
        ])
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        bag_exec,
        robot_state_publisher_launch,
        rviz_node,
        TimerAction(
            period=1.0, # delay these nodes for 1.0 seconds. It takes about 1 second for bag to start publish data.
            actions=[
                gps_imu_launch,
                ekf_localizer_launch
            ]
        )
    ])
