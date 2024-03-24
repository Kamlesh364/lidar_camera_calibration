from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from os.path import join


def generate_launch_description():
    # Setup params for Camera-LiDAR calibration script

    # Launch_args = [
    #     DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/rgb/camera_info'),
    #     DeclareLaunchArgument('camera_topic', default_value='/zed/zed_node/rgb/image_rect_color'),
    #     DeclareLaunchArgument('velodyne_points_topic', default_value='/velodyne_points'),
    #     DeclareLaunchArgument('lidar_topic', default_value='/fused_points'),
    #     DeclareLaunchArgument('bag_files', default_value='/workspaces/isaac_ros-dev/Lab/'),
    # ]

    # bag_files = LaunchConfiguration('bag_files')
    # rosbag_p1 = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_files], output='screen', log_cmd=False)

    tf_static_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_velodyne_tf',
        output='screen',
        arguments=['-0.10314803', '-0.03314803', '0.11614803', '0', '0', '-0.010', 'zed_camera_center', 'velodyne']
        # arguments=['0', '0', '0', '0', '0', '0', 'zed_camera_center', 'velodyne']
    )

    image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_node1'
    )

    fusion_node = Node(
        package='lidar_camera_calibration',
        executable='calibration_projection_node',
        name='calibration_projection_node',
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
        tf_static_lidar,
        image_proc,
        fusion_node
    ])

    return final_launch_description