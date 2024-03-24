# lidar_camera_calibration

**Description**: This ROS 2 package provides tools for calibrating LiDAR and camera systems and projecting LiDAR data onto camera images.

## Installation

Clone this repository into your ROS 2 workspace and build it using colcon:

```bash
cd ~/ros2_ws/src
git clone https://github.com/kamlesh364/lidar_camera_calibration.git
cd ..
colcon build
source install/setup.bash
```

## Dependencies

This package depends on the following ROS 2 packages:

- `velodyne` package for Velodyne LiDAR integration
- `zed_wrapper` package for ZED camera integration
- `image_proc` package for image processing

Make sure these dependencies are installed and available in your ROS 2 workspace.

## Usage

### Calibration Launch File

The package includes a launch file named `calibration.launch.py` which can be used to launch the calibration process. This launch file sets up the necessary nodes and parameters for calibrating the LiDAR and camera systems.

To use the launch file, run the following command:

```bash
ros2 launch lidar_camera_calibration calibration.launch.py
```

### Launch File Description

The launch file `calibration.launch.py` sets up the following nodes:

1. **Velodyne Node**: Launches the Velodyne LiDAR nodes required for data acquisition.
2. **ZED Node**: Launches the ZED camera nodes required for data acquisition.
3. **Static Transform Publisher**: Publishes a static transformation between the LiDAR frame and the camera frame.
4. **Image Proc Node**: Performs image processing on the camera image.
5. **Calibration Projection Node**: Projects LiDAR data onto the camera image.

### Parameters

The launch file allows you to configure the following parameters:

- `camera_info_topic`: Topic for camera information (default: `/zed/zed_node/rgb/camera_info`).
- `camera_topic`: Topic for camera image (default: `/zed/zed_node/rgb/image_rect_color`).
- `velodyne_points_topic`: Topic for Velodyne LiDAR points (default: `/velodyne_points`).
- `lidar_topic`: Topic for fused LiDAR points (default: `/fused_points`).
- `bag_files`: Path to ROS 2 bag files for data playback (optional).

## Nodes

### Calibration Projection Node

This node subscribes to the corrected Velodyne LiDAR point cloud data and the camera image. It then projects the LiDAR data onto the camera image and publishes the resulting image with LiDAR data overlaid to a new ROS topic.

#### Static Transform Publisher Node

You can use the `static_transform_publisher` node to publish the calibrated LiDAR point cloud data in the camera frame. Once you have obtained the calibration parameters between the LiDAR and camera frames, you can create a static transformation between these frames and publish it using the `static_transform_publisher` node.


1. **Obtain Calibration Parameters**: First, calibrate the LiDAR and camera systems to determine the transformation between their respective coordinate frames. This includes parameters such as translation and rotation.

2. **Create Static Transformation**: Use the calibration parameters to create a static transformation between the LiDAR frame and the camera frame. This transformation defines the relationship between the two frames.

3. **Publish Transformation**: Use the `static_transform_publisher` node to publish the static transformation between the LiDAR and camera frames. This makes the calibrated LiDAR point cloud data available in the camera frame within the TF system.

4. **Verify**: Verify that the transformation is correctly published by visualizing the LiDAR point cloud data in the camera frame using TF tools or visualization tools like RViz.

Here's an example launch file to publish the static transformation:

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="lidar_to_camera_tf" args="x y z yaw pitch roll parent_frame child_frame" />
</launch>
```

Replace `x`, `y`, `z`, `yaw`, `pitch`, `roll`, `parent_frame`, and `child_frame` with the appropriate values for your transformation. These values should be obtained from your calibration process.

By publishing the static transformation, you'll be able to access the LiDAR point cloud data in the camera frame, facilitating integration and further processing in applications such as sensor fusion, object detection, and mapping.

#### Published Topics

- `/projected_image`: The projected image with LiDAR data overlaid.

#### Subscribed Topics

- `/corrected_velodyne_points`: Corrected Velodyne LiDAR point cloud data.
- `/camera/image`: Camera image.

Here are the instructions for the `projection.launch.py` launch file:

### Projection Launch File

This package also includes a launch file named `projection.launch.py` which can be used to launch the projection process. This launch file sets up the necessary nodes and parameters for projecting LiDAR data onto the camera image.

To use the launch file, run the following command:

```bash
ros2 launch lidar_camera_calibration projection.launch.py
```

### Launch File Description

The launch file `projection.launch.py` sets up the following nodes:

1. **Static Transform Publisher**: Publishes a static transformation between the LiDAR frame and the camera frame.
2. **Image Proc Node**: Performs image processing on the camera image.
3. **Calibration Projection Node**: Projects LiDAR data onto the camera image.

### Parameters

The launch file allows you to configure the following parameters:

- `camera_info_topic`: Topic for camera information (default: `/zed/zed_node/rgb/camera_info`).
- `camera_topic`: Topic for camera image (default: `/zed/zed_node/rgb/image_rect_color`).
- `velodyne_points_topic`: Topic for Velodyne LiDAR points (default: `/velodyne_points`).
- `lidar_topic`: Topic for fused LiDAR points (default: `/fused_points`).
- `bag_files`: Path to ROS 2 bag files for data playback (optional).

### Nodes

#### Calibration Projection Node

This node subscribes to the corrected Velodyne LiDAR point cloud data and the camera image. It then projects the LiDAR data onto the camera image and publishes the resulting image with LiDAR data overlaid to a new ROS topic.

##### Published Topics

- `/projected_image`: The projected image with LiDAR data overlaid.

##### Subscribed Topics

- `/corrected_velodyne_points`: Corrected Velodyne LiDAR point cloud data.
- `/camera/image`: Camera image.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
