#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from image_geometry import PinholeCameraModel
import threading
from tf2_sensor_msgs import do_transform_cloud
import collections

class LidarCameraRegistrationNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(300))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.laser_projector = LaserProjection()

        self.lock = threading.Lock()

        self.cam_name = rospy.get_namespace().strip('/')
        self.image_queue = collections.deque()
        self.scan_queue = collections.deque()

        self.camera_frame = rospy.get_param('~camera_frame', f'{self.cam_name}_jai_optical_frame')
        self.target_frame = rospy.get_param('~target_frame', 'motor_odom')
        self.laser_frame = rospy.get_param('~laser_frame', f'{self.cam_name}_base_laser')

        self.point_cloud_motor_odom_pub = rospy.Publisher('/point_cloud_motor_odom', PointCloud2, queue_size=1)
        self.point_cloud_camera_pub = rospy.Publisher('/point_cloud_camera', PointCloud2, queue_size=1)
        self.depth_image_pub = rospy.Publisher('/depth_image', Image, queue_size=1)

        self.camera_model = None
        self.accumulated_cloud = []  # List of scans

        # Retention parameters
        self.removal_threshold_x = rospy.get_param('~removal_threshold_x', 5.0)  # Remove scans 5 meters behind the camera

        # Camera parameters
        self.image_width = None
        self.image_height = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.image_sub = rospy.Subscriber(f'/{self.cam_name}/bgri', Image, self.image_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=20)
        self.camera_info_sub = rospy.Subscriber(f'/{self.cam_name}/bgri/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        # Timer for periodic processing (e.g., every 0.5 seconds)
        self.processing_timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        rospy.loginfo("Lidar Camera Registration Node Initialized")

    def camera_info_callback(self, camera_info_msg):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)
        
        self.image_width = camera_info_msg.width
        self.image_height = camera_info_msg.height
        
        # Extract and store intrinsic parameters by calling the methods
        self.fx = self.camera_model.fx()
        self.fy = self.camera_model.fy()
        self.cx = self.camera_model.cx()
        self.cy = self.camera_model.cy()

    def scan_callback(self, scan_msg):
        with self.lock:
            self.scan_queue.append(scan_msg)

    def image_callback(self, image_msg):
        with self.lock:
            self.image_queue.append(image_msg)

    def timer_callback(self, event):
        with self.lock:
            self.process_data()

    def process_data(self):
        if not self.scan_queue and not self.image_queue:
            return

        # Ensure camera_model and camera parameters are initialized
        if (self.camera_model is None or
            self.image_width is None or
            self.image_height is None or
            self.fx is None or
            self.fy is None or
            self.cx is None or
            self.cy is None):
            rospy.logwarn("Camera parameters not initialized yet. Waiting for CameraInfo...")
            return

        self.accumulate_scans()
        self.remove_old_scans()  # Remove outdated scans

        if self.image_queue:
            self.process_images()

    def accumulate_scans(self):
        scans_processed = 0
        max_scans_per_cycle = 10

        while self.scan_queue and scans_processed < max_scans_per_cycle:
            scan_msg = self.scan_queue.popleft()
            scans_processed += 1
            cloud = self.laser_projector.projectLaser(scan_msg)
            try:
                transform_to_motor_odom = self.tf_buffer.lookup_transform(
                    self.target_frame, self.laser_frame, scan_msg.header.stamp, rospy.Duration(0.2))
                
                cloud_transformed = do_transform_cloud(cloud, transform_to_motor_odom)
                points = list(point_cloud2.read_points(cloud_transformed, field_names=("x", "y", "z"), skip_nans=True))
                
                if not points:
                    rospy.logwarn("No valid points in the transformed cloud.")
                    continue
                
                # Compute the maximum x-coordinate in the scan for removal criteria
                max_x = max(pt[0] for pt in points)
                
                # Append the entire scan
                self.accumulated_cloud.append({
                    'points': points,
                    'transform': transform_to_motor_odom,
                    'timestamp': scan_msg.header.stamp,
                    'max_x': max_x
                })
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform from laser frame to motor odom frame.")

        self.publish_accumulated_cloud(rospy.Time.now())

    def publish_accumulated_cloud(self, timestamp):
        if not self.accumulated_cloud:
            return

        all_points = []
        for scan in self.accumulated_cloud:
            all_points.extend(scan['points'])
        
        if not all_points:
            rospy.logwarn("No points to publish in accumulated_cloud.")
            return

        cloud_msg = point_cloud2.create_cloud_xyz32(
            rospy.Header(frame_id=self.target_frame, stamp=timestamp),
            all_points
        )
        self.point_cloud_motor_odom_pub.publish(cloud_msg)

    def process_images(self):
        if not self.image_queue or not self.accumulated_cloud:
            return 
        
        image_msg = self.image_queue[0]
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        latest_timestamp = image_msg.header.stamp

        points_2d, depths = self.project_scans_to_image(latest_timestamp)
        self.display_points_on_image(cv_image, points_2d)

        self.publish_accumulated_cloud(latest_timestamp)

        self.display_depth_images(cv_image, points_2d, depths)
        self.image_queue.popleft()

    def display_points_on_image(self, cv_image, points_2d):
        # Display projected points on the image for visualization
        for point in points_2d:
            u, v = int(point[0]), int(point[1])
            if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                cv2.circle(cv_image, (u, v), 3, (0, 0, 255), -1)

        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

    def remove_old_scans(self):
        if not self.accumulated_cloud:
            return

        removal_threshold = self.removal_threshold_x  # e.g., 5.0 meters

        scans_to_keep = []
        removed_count = 0

        for scan in self.accumulated_cloud:
            scan_time = scan['timestamp']
            try:
                # Lookup transform at the scan's timestamp
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, self.camera_frame, scan_time, rospy.Duration(0.2))

                # Extract only the x-coordinate to minimize overhead
                camera_x = transform.transform.translation.x

                # Vectorized condition check using simple arithmetic
                if scan['max_x'] >= (camera_x - removal_threshold):
                    scans_to_keep.append(scan)
                else:
                    removed_count += 1

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Failed to lookup camera transform at scan time {scan_time}: {e}")
                scans_to_keep.append(scan)

        self.accumulated_cloud = scans_to_keep

        if removed_count > 0:
            rospy.loginfo(f"Removed {removed_count} old scans based on x-coordinate relative to camera position at scan times.")

    def is_within_fov(self, point_camera_frame):
        # Determine if a point in the camera frame is within the camera's FOV
        u = (point_camera_frame[0] * self.fx / point_camera_frame[2]) + self.cx
        v = (point_camera_frame[1] * self.fy / point_camera_frame[2]) + self.cy

        # Check if the projected point is within the image boundaries
        return 0 <= u < self.image_width and 0 <= v < self.image_height

    def display_depth_images(self, cv_image, points_2d, depths):
        height, width, _ = cv_image.shape
        depth_image = np.zeros((height, width), dtype=np.float32)

        for (u, v), d in zip(points_2d, depths):
            if 0 <= int(v) < height and 0 <= int(u) < width:
                depth_image[int(v), int(u)] = d

        kernel = np.ones((3, 3), np.uint8)
        depth_image = cv2.dilate(depth_image, kernel, iterations=1)

        if np.any(depth_image > 0):
            depth_image_filtered = depth_image[depth_image > 0]
            depth_image_log = np.log1p(depth_image_filtered)
            depth_image_normalized = cv2.normalize(depth_image_log, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = np.uint8(depth_image_normalized)
            depth_image_colored = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_TURBO)
            depth_image_final = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
            mask = depth_image > 0
            depth_image_final[mask] = depth_image_colored.reshape(-1, 3)
        else:
            rospy.logwarn("Depth image is empty after filtering.")
            depth_image_final = np.zeros((height, width, 3), dtype=np.uint8)

        
        # publish depth image
        try:
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image_final, "bgr8")
            depth_image_msg.header.stamp = rospy.Time.now()
            self.depth_image_pub.publish(depth_image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        cv2.imshow('Depth Image', depth_image_final)
        cv2.waitKey(1)

    def project_scans_to_image(self, timestamp):
        # Transform scans from motor_odom to camera frame
        try:
            transform_to_camera = self.tf_buffer.lookup_transform(
                self.camera_frame, self.target_frame, timestamp, rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Lookup Error: {e}")
            return [], []

        cloud_transformed = do_transform_cloud(
        point_cloud2.create_cloud_xyz32(
            rospy.Header(frame_id=self.target_frame, stamp=timestamp),
            [pt for scan in self.accumulated_cloud for pt in scan['points']]),
        transform_to_camera)

        cloud_transformed.header.stamp = timestamp
        cloud_transformed.header.frame_id = self.camera_frame
        self.point_cloud_camera_pub.publish(cloud_transformed)

        points_camera_frame = np.array(list(point_cloud2.read_points(cloud_transformed, field_names=("x", "y", "z"), skip_nans=True)))

        # Filter points within FOV
        if points_camera_frame.size == 0:
            return [], []

        # Identify indices of points within FOV
        within_fov_indices = [i for i, point in enumerate(points_camera_frame) if point[2] > 0 and self.is_within_fov(point)]
        if not within_fov_indices:
            return [], []

        points_within_fov = points_camera_frame[within_fov_indices]
        points_2d, depths = self.project_points(points_within_fov)

        return points_2d, depths

    def project_points(self, points_camera_frame):
        # Vectorize projection of points
        z_positive = points_camera_frame[:, 2] > 0
        points_camera_frame = points_camera_frame[z_positive]
        u = (points_camera_frame[:, 0] * self.fx / points_camera_frame[:, 2]) + self.cx
        v = (points_camera_frame[:, 1] * self.fy / points_camera_frame[:, 2]) + self.cy
        depths = points_camera_frame[:, 2]
        points_2d = np.column_stack((u, v))
        return points_2d, depths

def main():
    rospy.init_node('lidar_camera_registration', anonymous=True)
    node = LidarCameraRegistrationNode()
    rospy.spin()

if __name__ == '__main__':
    main()