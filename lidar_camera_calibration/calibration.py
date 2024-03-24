#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from .utils.ros2_numpy import point_cloud2_to_xyz_array

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.lidar_subscriber = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10)
        self.camera_subscriber = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.camera_callback,
            10)
        self.lidar_points = None
        self.camera_points = None
        self.selected_lidar_points = []
        self.selected_camera_points = []
        self.selecting_points = False

    def lidar_callback(self, msg):
        # Convert PointCloud2 to numpy array
        lidar_points = point_cloud2_to_xyz_array(msg)
        self.lidar_points = lidar_points

    def camera_callback(self, msg):
        # Convert PointCloud2 to numpy array
        camera_points = np.array([ [p.x, p.y, p.z] for p in msg.data], dtype=np.float32)
        self.camera_points = camera_points

    def visualize_points(self):
        if self.lidar_points is None or self.camera_points is None:
            return
        
        self.selecting_points = True

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.selected_lidar_points.append(self.lidar_points[y, x])
                self.selected_camera_points.append(self.camera_points[y, x])

        cv2.namedWindow('Visualization')
        cv2.setMouseCallback('Visualization', mouse_callback)

        while self.selecting_points:
            # Visualize lidar and camera points
            # You can use OpenCV or any other visualization library here
            visualization = np.zeros((600, 800, 3), dtype=np.uint8)
            cv2.imshow('Visualization', visualization)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

    def calculate_calibration_parameters(self):
        if len(self.selected_lidar_points) == 0 or len(self.selected_camera_points) == 0:
            self.get_logger().info("No points selected. Cannot calculate calibration parameters.")
            return

        # Assuming selected points are corresponding points on the checkerboard
        lidar_points = np.array(self.selected_lidar_points)
        camera_points = np.array(self.selected_camera_points)

        # Calculate translation and rotation matrices
        translation = np.mean(camera_points - lidar_points, axis=0)
        rotation_matrix = np.dot(np.linalg.inv(lidar_points.T @ lidar_points), lidar_points.T @ camera_points)

        # Convert rotation matrix to Euler angles
        r = R.from_matrix(rotation_matrix)
        euler_angles = r.as_euler('zyx', degrees=True)

        self.get_logger().info("Translation: %s" % translation)
        self.get_logger().info("Rotation Matrix: %s" % rotation_matrix)
        self.get_logger().info("Euler Angles (degrees): %s" % euler_angles)

        # Save data in .npz format
        np.savez('calibration_data/calibration_metrics.npz', translation=translation, rotation_matrix=rotation_matrix, euler_angles=euler_angles)

def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()
    try:
        calibration_node.visualize_points()
        calibration_node.calculate_calibration_parameters()
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        calibration_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
