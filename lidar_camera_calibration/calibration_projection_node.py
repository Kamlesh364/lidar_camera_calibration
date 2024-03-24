import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from .utils.image_geometry import PinholeCameraModel
from .utils.ros2_numpy import point_cloud2_to_xyz_array

class CalibrationProjectionNode(Node):
    def __init__(self):
        super().__init__('calibration_projection_node')
        self.subscription_lidar = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10)
        self.subscription_image = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            10)
        self.publisher_projected_image = self.create_publisher(
            Image,
            '/fused_image',
            10)
        self.bridge = CvBridge()
        self.lidar_points = None
        self.camera_image = None
        self.camera_info = None
        self.pinhole_camera_model = PinholeCameraModel()

    def lidar_callback(self, msg):
        # Process LiDAR data
        # For simplicity, we're just storing the point cloud
        self.lidar_points = point_cloud2_to_xyz_array(msg)

    def image_callback(self, msg):
        # Process camera image
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        # Process camera info
        self.camera_info = msg
        # Update PinholeCameraModel with camera info
        self.pinhole_camera_model.fromCameraInfo(self.camera_info)

    def project_lidar_on_image(self):
        if self.lidar_points is None or self.camera_image is None or self.camera_info is None:
            return

        # Convert LiDAR point cloud to numpy array
        lidar_points = self.lidar_points

        # Project LiDAR points onto camera image using PinholeCameraModel
        projected_image = self.camera_image.copy()
        for point in lidar_points:
            # Project LiDAR point onto camera image
            # Use PinholeCameraModel for projection
            img_point = self.pinhole_camera_model.project3dToPixel(point)
            if self.pinhole_camera_model.rectifyPoint(point):
                projected_x = int(img_point[0])
                projected_y = int(img_point[1])
                # Check if pixel coordinates are within image boundaries
                if 0 <= projected_x < self.camera_image.shape[1] and 0 <= projected_y < self.camera_image.shape[0]:
                    # Draw LiDAR point on camera image
                    cv2.circle(projected_image, (projected_x, projected_y), 2, (0, 255, 0), -1)  # Placeholder: Draw circle

        # Publish the modified camera image with LiDAR points overlaid
        projected_image_msg = self.bridge.cv2_to_imgmsg(projected_image, encoding='bgr8')
        self.publisher_projected_image.publish(projected_image_msg)

def main(args=None):
    rclpy.init(args=args)
    projection_node = CalibrationProjectionNode()
    try:
        while True:
            projection_node.project_lidar_on_image()
            rclpy.spin_once(projection_node)
    except KeyboardInterrupt:
        projection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
