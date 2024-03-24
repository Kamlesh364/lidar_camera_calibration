from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from os.path import join


def generate_launch_description():
    # Setup params for Camera-LiDAR calibration script

    # Launch_args = [
    #     DeclareLaunchArgument('bag_files', default_value='/workspaces/isaac_ros-dev/Lab/'),
    # ]

    # bag_files = LaunchConfiguration('bag_files')

    # rosbag_p1 = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_files], output='screen', log_cmd=False)

    image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_node1'
    )

    calibration_node = Node(
        package='lidar_camera_calibration',
        executable='calibration',
        name='calibration_node',
        output='log',
        log_cmd=True
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(get_package_share_directory('velodyne'),'launch/velodyne-all-nodes-VLP16-launch.py')),
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(get_package_share_directory('zed_wrapper'),'launch/obsolete/zed.launch.py'))
    )

    final_launch_description = LaunchDescription([
        # *Launch_args,
        velodyne_launch,
        zed_launch,
        # rosbag_p1,
        image_proc,
        calibration_node
    ])

    return final_launch_description